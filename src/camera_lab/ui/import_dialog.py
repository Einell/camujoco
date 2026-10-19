import math
from PySide6.QtWidgets import QDialog,QFormLayout,QDoubleSpinBox,QDialogButtonBox


class PlacementDialog(QDialog):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setWindowTitle('Append model placement')
        layout=QFormLayout(self)
        self.fields=[]
        for label in ('X (m)','Y (m)','Z (m)','Roll (degrees)','Pitch (degrees)','Yaw (degrees)'):
            field=QDoubleSpinBox()
            field.setRange(-10000,10000)
            field.setDecimals(4)
            field.setSingleStep(.1 if len(self.fields)<3 else 5)
            self.fields.append(field);layout.addRow(label,field)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept);buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def placement(self):
        values=[field.value() for field in self.fields]
        roll,pitch,yaw=[math.radians(value)/2 for value in values[3:]]
        cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
        # Fixed-axis XYZ roll/pitch/yaw, equivalent to Rz(yaw) Ry(pitch) Rx(roll).
        quat=(cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)
        return values[:3],quat
