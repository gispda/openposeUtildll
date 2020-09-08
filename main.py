#import _openposeUtil
import ctypes
import os.path

# _openposeUtil.zedservice_connectToVideo()
# _openposeUtil.zedservice_startSaveposeavi()
# _openposeUtil.zedservice_stopProcessingThread()
import sys

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QFileDialog
import pymain

class MyWindow(QtWidgets.QWidget):
    def __init__(self):
        super(MyWindow,self).__init__()
        self.myButton = QtWidgets.QPushButton(self)
        self.myButton.setObjectName("myButton")
        self.myButton.setText("Test")
        self.myButton.clicked.connect(self.msg)

    def msg(self):
        fileName1, filetype = QFileDialog.getOpenFileName(self,
                                    "选取文件",
                                    "./",
                                    "All Files (*);;avi Files (*.*)")   #设置文件扩展名过滤,注意用双分号间隔
        print(fileName1,filetype)



def click_connectVideo():
       
    
    dll_name = "openposeutil.dll"
    dllabspath = os.path.dirname(os.path.abspath(__file__)) + os.path.sep + dll_name
    
    myDll = ctypes.CDLL(dllabspath)
    
    myDll.startposeservice.argtypes = [ctypes.c_char_p]
    myDll.startposeservice.restype = ctypes.c_void_p
    
    
    svofiles=bytes("D:\\project\\ParatroopersTraining\\data\\svo.svo","gbk")    
    
    myDll.startposeservice(svofiles,True)
    
    print("----------------")
    #myDll.detach()
    #pyResult=ctypes.string_at(result);
	#print(pyResult.decode("gbk"))

    #_openposeUtil.zedservice_startposeservice('D:\\project\\ParatroopersTraining\\data\\222.svo')
def click_getavi_file():
    dll_name = "openposeutil.dll"
    dllabspath = os.path.dirname(os.path.abspath(__file__)) + os.path.sep + dll_name
    
    myDll = ctypes.CDLL(dllabspath)
    myDll.getposeavifile.argtypes=[]

    myDll.getposeavifile.restype=ctypes.c_char_p
    poseavi = myDll.getposeavifile()
    print(poseavi)

def click_getposedata():
    dll_name = "openposeutil.dll"
    dllabspath = os.path.dirname(os.path.abspath(__file__)) + os.path.sep + dll_name
    
    myDll = ctypes.CDLL(dllabspath)
    myDll.getposedatadir.argtypes=[]

    myDll.getposedatadir.restype=ctypes.c_char_p
    poseavi = myDll.getposedatadir()
    print(poseavi)

def click_stopVideo():
    #print("啊哈哈哈我终于成功了！")
    dll_name = "openposeutil.dll"
    dllabspath = os.path.dirname(os.path.abspath(__file__)) + os.path.sep + dll_name
    
    myDll = ctypes.CDLL(dllabspath)
    
    #myDll.endposeservice.argtypes = []
    myDll.endposeservice.restype = ctypes.c_void_p
    myDll.endposeservice()


    #_openposeUtil.zedservice_stopposeservice()
#def click_startposeavi():
    #print("啊哈哈哈我终于成功了！")
    #_openposeUtil.zedservice_startgetposeavidata()

#def click_endposeavi():
    #print("啊哈哈哈我终于成功了！")
    #_openposeUtil.zedservice_endSaveposeavi()

#def click_startsavezedavi():
    #print("啊哈哈哈我终于成功了！")
    #_openposeUtil.zedservice_startSavezedavi()

#def click_endzedavi():
    #print("啊哈哈哈我终于成功了！")
    #_openposeUtil.zedservice_endgetposeavidata()

def click_startmergeavi():
 #   print("------------")
    #_openposeUtil.zedservice_sethkFile('D:\\project\ParatroopersTraining\\data\\ch0001_00000000507000106.mp4')
    #print("11111111111")
    #_openposeUtil.zedservice_setzedFile('D:\\project\ParatroopersTraining\\data\\202007280659010041.avi')
    #print("12222222")
    #_openposeUtil.zedservice_setposeFile('D:\\project\ParatroopersTraining\\data\\202007262258180041.avi')
    #print("33333333333")
    #_openposeUtil.zedservice_setjsonFile('D:\\project\ParatroopersTraining\\data\\202007271001018467.json')
    #print("4444444444444")
    #_openposeUtil.zedservice_startmergereportavi()
    #_openposeUtil.zedservice_startmergereportavi('D:\\project\\ParatroopersTraining\\data\\ch0001_00000000507000106.mp4','D:\\project\\ParatroopersTraining\\data\\222.svo','D:\\project\\ParatroopersTraining\\openposeUtilswig\\2020080320001141.avi','D:\\project\\ParatroopersTraining\\openposeUtilswig\\xxx\\')
    dll_name = "openposeutil.dll"
    dllabspath = os.path.dirname(os.path.abspath(__file__)) + os.path.sep + dll_name
    
    myDll = ctypes.CDLL(dllabspath)
    
    myDll.startmergereportavi.argtypes = [ctypes.c_char_p,ctypes.c_char_p,ctypes.c_char_p,ctypes.c_char_p]
    myDll.startmergereportavi.restype = ctypes.c_char_p
    hkfiles=bytes("D:\\project\\ParatroopersTraining\\data\\ch0001_00000001349000306.mp4","gbk")   
    svofiles=bytes("D:\\project\\ParatroopersTraining\\data\\svo.svo","gbk")   
    posefiles=bytes("D:\\project\\ParatroopersTraining\\openposeUtildll\\test\\2020090811011391.avi","gbk")   
    datadir=bytes("D:\\project\\ParatroopersTraining\\openposeUtildll\\test\\2020090811011358\\","gbk")   
    cmpfile = myDll.startmergereportavi(hkfiles,svofiles,posefiles,datadir,True)
    
    print(cmpfile)
    print("over")
#def click_startrecordzedimg():
    #print("啊哈哈哈我终于成功了！")
    #_openposeUtil.zedservice_getzedcurimg()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = pymain.Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    ui.pushButton_5.clicked.connect(click_connectVideo)
    ui.pushButton_6.clicked.connect(click_stopVideo)
    ui.pushButton_7.clicked.connect(click_startmergeavi)
    ui.pushButton_8.clicked.connect(click_getavi_file)
    ui.pushButton_9.clicked.connect(click_getposedata)
    sys.exit(app.exec_())

