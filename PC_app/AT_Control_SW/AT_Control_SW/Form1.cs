using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;


namespace AT_Control_SW
{
    
    public partial class Form1 : Form
    {
        // #defines - start

        const byte TX_ID_START_STOP     = 0x11;
        const byte TX_ID_SPEED          = 0x12;
        const byte TX_ID_MODE           = 0X13;
        const byte TX_ID_DIR            = 0X14;
        const byte TX_ID_PULSE_CNT      = 0X15;
        const byte TX_ID_ZERO           = 0X16;
        const byte TX_ID_QUADRANT       = 0X17;
        const byte TX_ID_DEGREES        = 0X18;

        const byte SOF = 0x55;
        const byte EOF = 0x5A;

//        const string comPort = "COM7";
        const string comPort = "COM3";

        const byte MODE_NORMAL          = 0;
        const byte MODE_SCAN            = 1;

        const byte ON                   = 1;
        const byte OFF                  = 0;

        const byte SET                  = 1;
        const byte CLEAR                = 0;

        const byte TRUE                 = 1;
        const byte FALSE                = 0;

        const byte CW = 0;
        const byte CCW = 1;

        const UInt16 SMALLEST_STEP_COUNT = 20;

        // 1 motor step = 0.9 deg of worm = 0.0225 deg of wheel
        // 1 clock pulse = 1 step
        // hence 1 deg = 44.4 pulses
        // See Comm protocol . xls file for calc
        // *** practical observations -> 1 deg = 100 pulses
        const UInt16 DEG_TO_PULSE_FACTOR = 100;

        const float V_STEP_SCAN_DEG = 0.5f;
        const UInt16 V_STEP_SCAN_PULSES = (UInt16)(V_STEP_SCAN_DEG * DEG_TO_PULSE_FACTOR);

        // #defines - end

        byte nextTxFrameID = TX_ID_START_STOP;

        byte sysState = OFF;
        byte sysMode = MODE_NORMAL;
        byte moveRequest = OFF;
        byte scanStartRequest = FALSE;
        
        byte sysSpeedRef = 3;
        byte sysDirecH = CW;
        byte sysDirecV = CW;
        UInt16 hPulseTargetCount = 0;
        UInt16 vPulseTargetCount = 0;
        UInt16 vDegreeTargetCount = 0;
        UInt16 hDegreeTargetCount = 0;
        byte zeroRequest = CLEAR;
        byte quadrantSet = 1;

        UInt16 stepCount = SMALLEST_STEP_COUNT;

        UInt16 mechPlayCompensator = 14 * SMALLEST_STEP_COUNT;

        // rx feedback from mount
        UInt16 fbHDir = CW;
        UInt16 fbVDir = CW;
        UInt16 fbSpeed = 0;
        UInt16 fbMode = MODE_NORMAL;
        UInt16 fbStatus = OFF;

        UInt16 fbHPulseElapsed = 0;
        UInt16 fbVPulseElapsed = 0;

        UInt16 fbQuad = 1;

        UInt16 fbHDegElapsed = 0;
        UInt16 fbVDegElapsed = 0;

        UInt16 rxDataRead = FALSE;

        float pbTarget = 0;
        float pbPresent = 0;
        float pbRatio = 0;

        UInt16 vScanCounter = 0;
        UInt16 hScanCounter = 0;

        UInt16 vScanTarget = 0;
        UInt16 hScanTarget = 0;

        UInt16 scanState = 0;
        UInt16 scanDelayTick = 0;

        bool scanProgress = false;

//        byte[,] scanDirArray = new byte[4, 3] { { CW, CW, CCW }, { CCW, CW, CW }, { CCW, CCW, CW }, { CW, CCW, CCW } };

        char[] rxBytes = new char[8];
        string rxString = new string('0', 8);

        public Form1()
        {
            InitializeComponent();

            InitParams();

            timer100.Start();
            timerSerialComm.Start();
        }

        void timer100_Tick(object sender, EventArgs e)
        {

            if (rbModeNormal.Checked)
            {
                btnZero.Enabled = false;
                //sysMode = MODE_NORMAL;
            }
            else
            {
                btnZero.Enabled = true;
                //sysMode = MODE_SCAN;
            }



            switch (tbResolution.Value)
            {
                case 0: stepCount = 1000; break;
                case 1: stepCount = 800; break;
                case 2: stepCount = 600; break;
                case 3: stepCount = 500; break;
                case 4: stepCount = 400; break;
                case 5: stepCount = 200; break;
                case 6: stepCount = 100; break;
                case 7: stepCount = SMALLEST_STEP_COUNT * 3; break;
                case 8: stepCount = SMALLEST_STEP_COUNT * 2; break;
                case 9: stepCount = SMALLEST_STEP_COUNT * 1; break;

            }


            sysSpeedRef = (byte)trackBar1.Value;

            if (serialPort1.IsOpen)
            {

                if (rxDataRead == TRUE)
                {


                    rxBytes = rxString.ToCharArray();

                    if (rxString.Length > 0)
                    {
                        // Communication process Rx
                        rvReadRequest();

                        rxDataRead = FALSE;
                    }
                }

            }


            scanManager();

            // update fb from mount on the status strip
            updateMountFeedback();


            label10.Text = fbQuad.ToString();


        }

        void scanManager()
        {

            // If scaner started
//            if (scanProgress)
            if (rbModeScanner.Checked)
            {
                switch (scanState)
                {
                    case 0 :    // H scan

                        if (scanDelayTick > 0)
                        {
                            scanDelayTick--;
                        }

                        if (0 == scanDelayTick)
                        {

                            if (vScanCounter > 0)
                            {
                                hPulseTargetCount = hScanTarget;
                                vPulseTargetCount = 0;

                                hPulseTargetCount += mechPlayCompensator;

                                vScanCounter--;

                                scanDirectionHandler(scanState);

                                moveRequest = SET;

                                // last line of scan
                                if (0 == vScanCounter)
                                {
                                    scanState = 2;
                                }
                                else // else continue scan
                                {
                                    scanState = 1;
                                }

                                scanDelayTick = 30;
                            }
                        }
                        break;

                    case 1:

                        if(scanDelayTick > 0)
                        {
                            scanDelayTick--;
                        }

                        if (0 == scanDelayTick)
                        {

                            if (fbHPulseElapsed == 0)
                            {
                                vPulseTargetCount = V_STEP_SCAN_PULSES;
                                hPulseTargetCount = 0;

                                scanDirectionHandler(scanState);

                                moveRequest = SET;

                                scanState = 0;

                                scanDelayTick = 30;
                            }
                        }

                        break;

                    case 2:

                        resetScanVar();

                        break;


                    default: break;
                }
            }
        }

        void resetScanVar()
        {
            //reset scan variables
            scanProgress = false;

            hScanTarget = 0;
            vScanTarget = 0;
            vScanCounter = 0;
            scanDelayTick = 0;

            scanState = 0;

            enDisDirecBtns(true);
        }

        void scanDirectionHandler(UInt16 scState)
        {
            switch(scState)
            {

                case 0:     // horizontal direction

                    if ((rbQuad1.Checked) || (rbQuad4.Checked))
                    {
                        // if odd line is to be scanned
                        if ((vScanCounter % 2) != 0)
                        {
                            sysDirecH = CW;
                        }
                        else
                        {
                            sysDirecH = CCW;
                        }
                    }
                    else if ((rbQuad2.Checked) || (rbQuad3.Checked))
                    {
                        // if odd line is to be scanned
                        if ((vScanCounter % 2) != 0)
                        {
                            sysDirecH = CCW;
                        }
                        else
                        {
                            sysDirecH = CW;
                        }
                    }


            break;

                case 1:     // vertical direction

                    if ((rbQuad1.Checked) || (rbQuad2.Checked))
                    {
                        sysDirecV = CW;
                    }
                    else if ((rbQuad3.Checked) || (rbQuad4.Checked))
                    {
                        sysDirecV = CCW;
                    }

            break;

                default:
            break;
            }
        }

        void updateMountFeedback()
        {



            // Status display
            if (ON == fbStatus)
            {
                ssStatus.Text = "ON";
                ssStatus.BackColor = Color.GreenYellow;
            }
            else if (OFF == fbStatus)
            {
                ssStatus.Text = "OFF";
                ssStatus.BackColor = DefaultBackColor;
            }

            // Mode display
            if (MODE_NORMAL == fbMode)
            {
                ssMode.Text = "Normal";
            }
            else if (MODE_SCAN == fbMode)
            {
                ssMode.Text = "Scan";
            }


            textBox3.Text = pbPresent.ToString();
            textBox4.Text = pbTarget.ToString();

            textBox5.Text = pbRatio.ToString();


            if ((pbTarget > 0) && (pbPresent > 0))
            {
                pbRatio = (((pbTarget - pbPresent) / pbTarget) * 100);
            }
            else
            {
                pbRatio = 0;
            }

            ssProgressBar.Value = (UInt16)pbRatio;


        }


        int rvReadRequest()
        {
            int rxStatus = -1;
            byte[] d = {0,0,0,0};
            UInt16 i;

            UInt16 rxSof = (UInt16)rxBytes[0];
            UInt16 rxFrameID = (UInt16)rxBytes[1];
            UInt16 rxDLC = (UInt16)rxBytes[2];
            UInt16 rxCkhSum = 0;

            for (i = 0; i < rxDLC; i++)
            {
                d[i] = (byte)rxBytes[ 3 + i ];
            }

            rxCkhSum = rxBytes[i];
            
            if (rxSof == SOF)
            {
                switch (rxFrameID)
                {
                    case 0x21:

                        fbVDir = (UInt16)(((byte)d[0] & 0xF0) >> 4);
                        fbHDir = (UInt16)(((byte)d[0] & 0x0F) >> 0);

                        fbSpeed = d[2];

                        fbMode = (UInt16)(((byte)d[3] & 0x02) >> 1);
                        fbStatus = (UInt16)(((byte)d[3] & 0x01) >> 0);
       
                        break;

                    case 0x25:

                        fbVPulseElapsed = Convert.ToUInt16(((d[0] & 0x00FF) << 8) | (d[1] & 0x00FF));
                        fbHPulseElapsed = Convert.ToUInt16(((d[2] & 0x00FF) << 8) | (d[3] & 0x00FF));

                        if (MODE_NORMAL == sysMode)
                        {
                            pbPresent = fbVPulseElapsed + fbHPulseElapsed;
                        }


                        break;

                    case 0x27:

                        fbQuad = d[0];

                        break;
                }
            }


            return rxStatus;
        }

        void rvSendResponse()
        {
            byte txFrameID = 0;

            // Tx frame - State manager
            txFrameID = TxStateManager();


            // Tx frame - preparation and transmission
            TransmitTxFrame(txFrameID);
        }

        byte TxStateManager()
        {
            byte lu8Return = 0;

            // If move request received in either mode
            if (TRUE == moveRequest)
            {
                nextTxFrameID = TX_ID_SPEED;
                moveRequest = FALSE;
            }
            // If scan start request received in Scan mode
            else if ((TRUE == scanStartRequest) & (MODE_SCAN == sysMode))
            {
                nextTxFrameID = TX_ID_ZERO;
                scanStartRequest = FALSE;
            }

            lu8Return = nextTxFrameID;

            switch(nextTxFrameID)
            {
                case TX_ID_START_STOP:  nextTxFrameID = TX_ID_MODE;         break;
                case TX_ID_MODE:        nextTxFrameID = TX_ID_START_STOP;   break;
                case TX_ID_SPEED:       nextTxFrameID = TX_ID_DIR;          break;
                case TX_ID_DIR:         nextTxFrameID = TX_ID_PULSE_CNT;    break;
                case TX_ID_PULSE_CNT:   nextTxFrameID = TX_ID_START_STOP;   break;
                case TX_ID_ZERO:        nextTxFrameID = TX_ID_QUADRANT;     break;
                case TX_ID_QUADRANT:    nextTxFrameID = TX_ID_DEGREES;      break;
                case TX_ID_DEGREES:     nextTxFrameID = TX_ID_START_STOP;   break;
                default: break;
            }

            // retuen ID of the frame to be transmitted now
             return lu8Return;
        }

        void TransmitTxFrame(byte frameID)
        {
            byte[] txBuff = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
            byte[] txFrameDLC = { 0, 1, 1, 1, 1, 4, 1, 1, 4 };
            byte dlc = 0, chksum = 0;
            byte lu8Loop;

            txBuff[0] = SOF;
            txBuff[1] = frameID;

            dlc = txFrameDLC[frameID - 0x10];
            txBuff[2] = dlc;

            switch (frameID)
            {
                case TX_ID_START_STOP:
                    txBuff[3] = sysState;
                    break;

                case TX_ID_SPEED:
                    txBuff[3] = sysSpeedRef;
                    break;

                case TX_ID_MODE:
                    txBuff[3] = sysMode;
                    break;

                case TX_ID_DIR:
                    txBuff[3] = (byte)(((UInt16)(sysDirecV & 0x0F) << 4) | (UInt16)sysDirecH);
                    break;

                case TX_ID_PULSE_CNT:
                    txBuff[3] = (byte)((vPulseTargetCount & 0xFF00) >> 8);
                    txBuff[4] = (byte)((vPulseTargetCount & 0x00FF) >> 0);
                    txBuff[5] = (byte)((hPulseTargetCount & 0xFF00) >> 8);
                    txBuff[6] = (byte)((hPulseTargetCount & 0x00FF) >> 0);
                    break;

                case TX_ID_ZERO:
                    txBuff[3] = zeroRequest;
                    break;

                case TX_ID_QUADRANT:
                    txBuff[3] = quadrantSet;
                    break;

                case TX_ID_DEGREES:
                    txBuff[3] = (byte)((vDegreeTargetCount & 0xFF00) >> 8);
                    txBuff[4] = (byte)((vDegreeTargetCount & 0x00FF) >> 0);
                    txBuff[5] = (byte)((hDegreeTargetCount & 0xFF00) >> 8);
                    txBuff[6] = (byte)((hDegreeTargetCount & 0x00FF) >> 0);
                    break;

                default: break;
            }

            // Calculate checksum
            for (lu8Loop = 0; lu8Loop < (3 + dlc); lu8Loop++)
            {
                chksum += txBuff[lu8Loop];
            }

            txBuff[lu8Loop++] = chksum;
            txBuff[lu8Loop++] = EOF;

            // Tx request
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(txBuff, 0, lu8Loop);
            }
        }

        private void btnConnect_Click(object sender, EventArgs e)
        {

            if (serialPort1.IsOpen == false)
            {
                serialPort1.Open();

                if (serialPort1.IsOpen == true)
                {
                    // Indicate com connected
                    tsStatusComm.Text = "Connected";
                    tsStatusComm.BackColor = Color.GreenYellow;

                    btnConnect.Text = "Disconnect";

                    btnStart.Enabled = true;
                    
                    // clear the Tx Rx buffers
                    serialPort1.DiscardOutBuffer();
                    serialPort1.DiscardInBuffer();
                }
            }
            else if (serialPort1.IsOpen == true)
            {
                // clear the Tx Rx buffers
                serialPort1.DiscardOutBuffer();
                serialPort1.DiscardInBuffer();

                serialPort1.Close();

                // Indicate com connected
                tsStatusComm.Text = "Disconnected";
                tsStatusComm.BackColor = DefaultBackColor;

                btnConnect.Text = "Connect";

                //btnStart.Enabled = false;
            }

        }


        private void Form1_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (serialPort1.IsOpen == true)
            {
                serialPort1.Close();
                if (serialPort1.IsOpen == false)
                {
                    tsStatusComm.Text = "COM Close";
                }
            }
        }
        
        void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            // Read Rx string till EOF character received
            rxString = serialPort1.ReadLine();
//            serialPort1.DiscardInBuffer();

            rxDataRead = TRUE;

        }

        private void button1_Click(object sender, EventArgs e)
        {
            moveRequest = 1;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            scanStartRequest = 1;
            sysMode = MODE_SCAN;

        }

        private void btnStart_Click(object sender, EventArgs e)
        {
            sysState = ON;
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            sysState = OFF;
            resetScanVar();
        }

        private void timerSerialComm_Tick(object sender, EventArgs e)
        {
            rvSendResponse();
        }

        void InitParams()
        {
            // Init serial comm data - 
            serialPort1.PortName = comPort;
            serialPort1.NewLine = "Z";  // 0x05A

            serialPort1.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);

            // Indicate com connected
            tsStatusComm.Text = "Disconnected";
            tsStatusComm.BackColor = DefaultBackColor;


            // speed trackbar
            trackBar1.Value = 3;

            // textbox init
            textBox1.Text = "0";
            textBox2.Text = "0";
            textBox3.Text = "0";

            //combo boxes

            cbDegX.Items.Add(1);
            cbDegX.Items.Add(2);
            cbDegX.Items.Add(3);
            cbDegX.Items.Add(4);
            cbDegX.Items.Add(5);
            cbDegX.Items.Add(6);
            cbDegX.Items.Add(7);
            cbDegX.Items.Add(8);
            cbDegX.Items.Add(9);
            cbDegX.Items.Add(10);

            cbDegX.SelectedItem = 5;

            cbDegY.Items.Add(1);
            cbDegY.Items.Add(2);
            cbDegY.Items.Add(3);
            cbDegY.Items.Add(4);
            cbDegY.Items.Add(5);
            cbDegY.Items.Add(6);
            cbDegY.Items.Add(7);
            cbDegY.Items.Add(8);
            cbDegY.Items.Add(9);
            cbDegY.Items.Add(10);

            cbDegY.SelectedItem = 2;

            
        }

        private void btnUp_Click(object sender, EventArgs e)
        {
            sysDirecV = CW;
            moveRequest = SET;
            vPulseTargetCount = stepCount;
            hPulseTargetCount = 0;

            pbTarget = stepCount;
        }

        private void btnDown_Click(object sender, EventArgs e)
        {
            sysDirecV = CCW;
            moveRequest = SET;
            vPulseTargetCount = stepCount;
            hPulseTargetCount = 0;

            pbTarget = stepCount;
        }

        private void btnRight_Click(object sender, EventArgs e)
        {
            bool useComp = false;

            if (sysDirecH == CCW)
            {
                useComp = true;
            }

            sysDirecH = CW;
            moveRequest = SET;
            hPulseTargetCount = stepCount;
            vPulseTargetCount = 0;

            if (useComp)
            {
                hPulseTargetCount += mechPlayCompensator;
            }

            pbTarget = hPulseTargetCount;
        }

        private void btnLeft_Click(object sender, EventArgs e)
        {
            bool useComp = false;

            if (sysDirecH == CW)
            {
                useComp = true;
            }

            sysDirecH = CCW;
            moveRequest = SET;
            hPulseTargetCount = stepCount;
            vPulseTargetCount = 0;

            if (useComp)
            {
                hPulseTargetCount += mechPlayCompensator;
            }

            pbTarget = hPulseTargetCount;
        }

        private void Form_Keypress(object sender, KeyPressEventArgs e)
        {

            switch (e.KeyChar)
            {

                    // Direction keys
                case 'i':
                    btnUp.Focus();
                    btnUp.PerformClick();
                    break;

                case 'k':
                    btnDown.Focus();
                    btnDown.PerformClick();
                    break;

                case 'j':
                    btnLeft.Focus();
                    btnLeft.PerformClick();
                    break;

                case 'l':
                    btnRight.Focus();
                    btnRight.PerformClick();
                    break;

                case 's':

                    btnStart.Focus();
                    btnStart.PerformClick();

                    break;

                case 'x':

                    btnStop.Focus();
                    btnStop.PerformClick();

                    break;

                case 'c' :

                    btnConnect.Focus();
                    btnConnect.PerformClick();

                    break;

                default:

                    if ((e.KeyChar >= '0') && (e.KeyChar <= '9'))
                    {
                        tbResolution.Value = e.KeyChar - 48;
                    }
                    
                    break;
            }
        }

        private void btnZero_Click(object sender, EventArgs e)
        {
            UInt16 xDeg;
            UInt16 yDeg;

            xDeg = Convert.ToUInt16(cbDegX.SelectedItem);
            yDeg = Convert.ToUInt16(cbDegY.SelectedItem);

            if ((xDeg > 0) && (yDeg > 0))
            {
                hScanTarget = (UInt16)((int)xDeg * (int)DEG_TO_PULSE_FACTOR);
                vScanTarget = (UInt16)((int)yDeg * (int)DEG_TO_PULSE_FACTOR);
                vScanCounter = (UInt16)((float)yDeg / V_STEP_SCAN_DEG);
/*
                enDisDirecBtns(false);
                scanStartRequest = TRUE;

                if (rbModeScanner.Checked)
                {
                    sysMode = MODE_SCAN;
                }

                scanProgress = true;
*/
            }
            else
            {
                hScanTarget = 0;
                vScanTarget = 0;
            }

            pbTarget = hScanTarget * vScanCounter;
        }

        void enDisDirecBtns(bool value)
        {
            btnUp.Enabled = value;
            btnDown.Enabled = value;
            btnLeft.Enabled = value;
            btnRight.Enabled = value;
        }


    }

}
