#region imports
from FourBar_GUI import Ui_Form
from FourBarLinkage_MVC import FourBarLinkage_Controller
import PyQt5.QtGui as qtg
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw
import sys
import numpy as np
#endregion

#region class definitions
class MainWindow(Ui_Form, qtw.QWidget):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # Add min/max angle controls
        self.minAngleSpin = qtw.QDoubleSpinBox(self)
        self.minAngleSpin.setRange(0, 360)
        self.minAngleSpin.setValue(0)
        self.maxAngleSpin = qtw.QDoubleSpinBox(self)
        self.maxAngleSpin.setRange(0, 360)
        self.maxAngleSpin.setValue(180)
        self.horizontalLayout.addWidget(qtw.QLabel("Min Angle"))
        self.horizontalLayout.addWidget(self.minAngleSpin)
        self.horizontalLayout.addWidget(qtw.QLabel("Max Angle"))
        self.horizontalLayout.addWidget(self.maxAngleSpin)

        # Add mass, spring, damping controls
        self.massSpin = qtw.QDoubleSpinBox(self)
        self.massSpin.setRange(0.1, 100)
        self.massSpin.setValue(10)
        self.springSpin = qtw.QDoubleSpinBox(self)
        self.springSpin.setRange(0.1, 100)
        self.springSpin.setValue(10)
        self.dampSpin = qtw.QDoubleSpinBox(self)
        self.dampSpin.setRange(0.1, 100)
        self.dampSpin.setValue(5)
        self.horizontalLayout.addWidget(qtw.QLabel("Mass"))
        self.horizontalLayout.addWidget(self.massSpin)
        self.horizontalLayout.addWidget(qtw.QLabel("Spring k"))
        self.horizontalLayout.addWidget(self.springSpin)
        self.horizontalLayout.addWidget(qtw.QLabel("Damping c"))
        self.horizontalLayout.addWidget(self.dampSpin)

        # Add Simulate button
        self.simButton = qtw.QPushButton("Simulate", self)
        self.simButton.clicked.connect(self.runSimulation)
        self.horizontalLayout.addWidget(self.simButton)

        # Create controller
        widgets = [self.gv_Main, self.nud_InputAngle, self.lbl_OutputAngle_Val,
                   self.nud_Link1Length, self.nud_Link3Length, self.spnd_Zoom]
        self.FBL_C = FourBarLinkage_Controller(widgets)
        self.FBL_C.setupGraphics()
        self.FBL_C.buildScene()
        # Set initial valid link lengths
        self.nud_Link1Length.setValue(60.0)  # Example: 60 units for input link
        self.nud_Link3Length.setValue(70.0)  # Example: 70 units for output link

        self.prevDashpotLength = self.FBL_C.FBL_M.DashPot.getLength()
        self.lblDashpotForce = qtw.QLabel("Dashpot F: 0.0 N")
        self.verticalLayout.addWidget(self.lblDashpotForce)

        self.nud_Link1Length.valueChanged.connect(self.setInputLinkLength)
        self.nud_Link3Length.valueChanged.connect(self.setOutputLinkLength)
        self.spnd_Zoom.valueChanged.connect(self.setZoom)
        self.show()

    def setInputLinkLength(self):
        try:
            self.FBL_C.setInputLinkLength()
            self.FBL_C.FBL_M.moveLinkage(self.FBL_C.FBL_M.InputLink.enPt)
            self.FBL_C.FBL_V.scene.update()
        except Exception as e:
            print(f"Error updating input link: {e}")

    def setOutputLinkLength(self):
        try:
            self.FBL_C.setOutputLinkLength()
            self.FBL_C.FBL_M.moveLinkage(self.FBL_C.FBL_M.InputLink.enPt)
            self.FBL_C.FBL_V.scene.update()
        except Exception as e:
            print(f"Error updating output link: {e}")

    def setZoom(self):
        self.gv_Main.resetTransform()
        self.gv_Main.scale(self.spnd_Zoom.value(), self.spnd_Zoom.value())

    def checkInputAngleLimits(self, scenePos):
        angle = self.FBL_C.FBL_M.InputLink.AngleDeg()
        if self.minAngleSpin.value() <= angle <= self.maxAngleSpin.value():
            self.FBL_C.moveLinkage(scenePos)
            self.updateDashpotForce()
        else:
            print("Input angle out of bounds!")

    def updateDashpotForce(self):
        currLength = self.FBL_C.FBL_M.DashPot.getLength()
        deltaL = currLength - self.prevDashpotLength
        rate = deltaL / 0.01  # small timestep
        dashpotF = self.dampSpin.value() * rate
        self.lblDashpotForce.setText(f"Dashpot F: {dashpotF:.2f} N")
        self.prevDashpotLength = currLength

    def runSimulation(self):
        self.sim_theta = np.pi       # start at 180°
        self.sim_omega = 0           # zero initial speed
        self.sim_dt = 0.01
        self.sim_mass = self.massSpin.value()
        self.sim_k = self.springSpin.value()
        self.sim_c = self.dampSpin.value()

        self.sim_timer = qtc.QTimer()
        self.sim_timer.timeout.connect(self.simulationStep)
        self.sim_timer.start(10)     # every 10 ms

    def simulationStep(self):
        torque = -self.sim_k * (self.sim_theta - np.pi/2) - self.sim_c * self.sim_omega
        alpha = torque / self.sim_mass
        self.sim_omega += alpha * self.sim_dt
        self.sim_theta += self.sim_omega * self.sim_dt

        self.FBL_C.FBL_M.InputLink.angle = self.sim_theta
        self.FBL_C.FBL_M.setInputLength(self.nud_Link1Length.value())
        self.FBL_C.FBL_M.moveLinkage(self.FBL_C.FBL_M.InputLink.enPt)
        self.FBL_C.FBL_V.scene.update()

        if abs(self.sim_theta - np.pi/2) < 0.01 and abs(self.sim_omega) < 0.01:
            self.sim_timer.stop()
            print("Simulation ended at equilibrium.")

#endregion

#region function calls
if __name__ == '__main__':
    app = qtw.QApplication(sys.argv)
    mw = MainWindow()
    mw.setWindowTitle('Four Bar Linkage')
    sys.exit(app.exec())
#endregion
