'''
Created on 2013-6-11

@author: Walter
'''

import matplotlib.pyplot as plt

if __name__ == '__main__':
    
    B_T = []
    E_T = []
    ABE_TTT = []
    ABE_TTF = []
    ABE_TFT = []
    ABE_TFF = []
    JA_TT = []
    JA_TF = []
    MA_TT = []
    MA_TF = []
    
    for line in open('alarm3-pars.txt'):
        exec(line)
        
    B_T1 = B_T
    E_T1 = E_T
    ABE_TTT1 = ABE_TTT
    ABE_TTF1 = ABE_TTF
    ABE_TFT1 = ABE_TFT
    ABE_TFF1 = ABE_TFF
    JA_TT1 = JA_TT
    JA_TF1 = JA_TF
    MA_TT1 = MA_TT
    MA_TF1 = MA_TF
    
    for line in open('alarm8-pars.txt'):
        exec(line)
        
    B_T2 = B_T
    E_T2 = E_T
    ABE_TTT2 = ABE_TTT
    ABE_TTF2 = ABE_TTF
    ABE_TFT2 = ABE_TFT
    ABE_TFF2 = ABE_TFF
    JA_TT2 = JA_TT
    JA_TF2 = JA_TF
    MA_TT2 = MA_TT
    MA_TF2 = MA_TF
    
    for line in open('alarm9-pars.txt'):
        exec(line)
        
    B_T3 = B_T
    E_T3 = E_T
    ABE_TTT3 = ABE_TTT
    ABE_TTF3 = ABE_TTF
    ABE_TFT3 = ABE_TFT
    ABE_TFF3 = ABE_TFF
    JA_TT3 = JA_TT
    JA_TF3 = JA_TF
    MA_TT3 = MA_TT
    MA_TF3 = MA_TF
    
    for line in open('alarm10-pars.txt'):
        exec(line)
        
    B_T4 = B_T
    E_T4 = E_T
    ABE_TTT4 = ABE_TTT
    ABE_TTF4 = ABE_TTF
    ABE_TFT4 = ABE_TFT
    ABE_TFF4 = ABE_TFF
    JA_TT4 = JA_TT
    JA_TF4 = JA_TF
    MA_TT4 = MA_TT
    MA_TF4 = MA_TF   
    
    label1 = "100 samples"
    label2 = "200 samples"
    label3 = "300 samples"
    label4 = "500 samples"
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.hist(B_T1, 50, normed=True, color='b', label=label1)
    ax1.hist(B_T2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax1.hist(B_T3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax1.hist(B_T4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax1.set_title("Burglary:T")
    ax1.legend()
    
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.hist(E_T1, 50, normed=True, color='b', label=label1)
    ax2.hist(E_T2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax2.hist(E_T3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax2.hist(E_T4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax2.set_title("Earthquake:T")
    ax2.legend()
    
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.hist(ABE_TTT1, 50, normed=True, color='b', label=label1)
    ax3.hist(ABE_TTT2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax3.hist(ABE_TTT3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax3.hist(ABE_TTT4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax3.set_title("Alarm:T|Burglary:T,Earthquake:T")
    ax3.legend()
    
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.hist(ABE_TTF1, 50, normed=True, color='b', label=label1)
    ax4.hist(ABE_TTF2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax4.hist(ABE_TTF3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax4.hist(ABE_TTF4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax4.set_title("Alarm:T|Burglary:T,Earthquake:F")
    ax4.legend() 

    fig5 = plt.figure()
    ax5 = fig5.add_subplot(111)
    ax5.hist(ABE_TFT1, 50, normed=True, color='b', label=label1)
    ax5.hist(ABE_TFT2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax5.hist(ABE_TFT3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax5.hist(ABE_TFT4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax5.set_title("Alarm:T|Burglary:F,Earthquake:T")
    ax5.legend() 
    
    fig6 = plt.figure()
    ax6 = fig6.add_subplot(111)
    ax6.hist(ABE_TFF1, 50, normed=True, color='b', label=label1)
    ax6.hist(ABE_TFF2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax6.hist(ABE_TFF3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax6.hist(ABE_TFF4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax6.set_title("Alarm:T|Burglary:F,Earthquake:F")
    ax6.legend()     
    
    fig7 = plt.figure()
    ax7 = fig7.add_subplot(111)
    ax7.hist(JA_TT1, 50, normed=True, color='b', label=label1)
    ax7.hist(JA_TT2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax7.hist(JA_TT3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax7.hist(JA_TT4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax7.set_title("MaryCalls:T|Alarm:T")
    ax7.legend()  
    
    fig8 = plt.figure()
    ax8 = fig8.add_subplot(111)
    ax8.hist(JA_TF1, 50, normed=True, color='b', label=label1)
    ax8.hist(JA_TF2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax8.hist(JA_TF3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax8.hist(JA_TF4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax8.set_title("MaryCalls:T|Alarm:F")
    ax8.legend() 
    
    fig9 = plt.figure()
    ax9 = fig9.add_subplot(111)
    ax9.hist(MA_TT1, 50, normed=True, color='b', label=label1)
    ax9.hist(MA_TT2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax9.hist(MA_TT3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax9.hist(MA_TT4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax9.set_title("JohnCalls:T|Alarm:T")
    ax9.legend()      

    fig10 = plt.figure()
    ax10 = fig10.add_subplot(111)
    ax10.hist(MA_TF1, 50, normed=True, color='b', label=label1)
    ax10.hist(MA_TF2, 50, normed=True, color='r', alpha=0.5, label=label2)
    ax10.hist(MA_TF3, 50, normed=True, color='g', alpha=0.5, label=label3)
    ax10.hist(MA_TF4, 50, normed=True, color='y', alpha=0.5, label=label4)
    ax10.set_title("JohnCalls:T|Alarm:F")
    ax10.legend()   
    
    plt.show()

    
    
    
    