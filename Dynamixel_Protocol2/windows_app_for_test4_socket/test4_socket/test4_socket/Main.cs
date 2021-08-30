using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using OpenJigWare;

namespace test4_socket
{
    public partial class Main : Form
    {
        public Main()
        {
            InitializeComponent();
        }

        Ojw.CProtocol2 m_CCom = new Ojw.CProtocol2();
        private void btnOpen_Click(object sender, EventArgs e)
        {
            if (m_CCom.IsOpen())
            {
                m_CCom.Close();
                btnOpen.Text = "Open";
                MessageBox.Show("통신이 닫혔습니다.");
            }
            else
            {
                int nPort = Ojw.CConvert.StrToInt(txtPort.Text);
                string strIP = txtIp.Text;
                if (m_CCom.Open(nPort, strIP) == true)
                {
                    MessageBox.Show("통신이 열렸습니다.");
                    btnOpen.Text = "Close";
                }
            }
        }

        private void Main_Load(object sender, EventArgs e)
        {
            txtMotion.Text = "s2,1000,0,1:90,2:90\r\n" +
                            "s2,1000,0,1:-90,2:-90\r\n" + 
                            "s2,1000,0,1:0,2:0";
        }

        private void btnPlay_Click(object sender, EventArgs e)
        {
            btnPlay.Enabled = false;
            string str = Ojw.CConvert.RemoveChar(txtMotion.Text, '\r');
            string[] pstr = str.Split('\n');
            foreach(string strLine in pstr)
            {
                m_CCom.Play_Stream(strLine);
                m_CCom.Wait();
            }
            btnPlay.Enabled = true;
            MessageBox.Show("모션이 끝났습니다.");
        }
    }
}
