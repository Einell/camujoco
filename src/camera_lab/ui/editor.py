"""Properties UI consumes copied descriptions, never a live MuJoCo model."""
from PySide6.QtCore import Signal, Qt
from PySide6.QtWidgets import (QWidget,QVBoxLayout,QFormLayout,QTreeWidget,QTreeWidgetItem,
    QComboBox,QDoubleSpinBox,QSpinBox,QPushButton,QLabel,QScrollArea)
from ..contracts import CameraConfig


class Editor(QWidget):
    command=Signal(str,object)

    def __init__(self):
        super().__init__()
        layout=QVBoxLayout(self)
        self.tree=QTreeWidget();self.tree.setHeaderLabels(['Scene / cameras'])
        layout.addWidget(self.tree)
        self.note=QLabel('Position: parent frame; free body: world.\nCamera axes: right/down/forward. Rotation: wxyz.')
        self.note.setWordWrap(True);layout.addWidget(self.note)
        form_widget=QWidget();form=QFormLayout(form_widget)
        scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(form_widget);layout.addWidget(scroll,1)
        self.parent=QComboBox();form.addRow('Camera parent',self.parent)
        self.values={}
        for key in ('x','y','z','qw','qx','qy','qz','width','height','fx','fy','cx','cy','near','far'):
            box=QSpinBox() if key in ('width','height') else QDoubleSpinBox()
            if key in ('width','height'):box.setRange(1,4096)
            else:box.setDecimals(6);box.setRange(-100000,100000);box.setSingleStep(.05)
            self.values[key]=box;form.addRow(key,box)
        self.apply=QPushButton('Apply · pause simulation');layout.addWidget(self.apply)
        self.target=QComboBox();layout.addWidget(self.target)
        self.aim=QPushButton('Aim camera at body');layout.addWidget(self.aim)
        self.apply.clicked.connect(self.submit)
        self.aim.clicked.connect(self.aim_selected)
        self.tree.currentItemChanged.connect(self.select)
        self.description=None;self.selected=None

    def update_scene(self,description):
        old=self.selected
        self.description=description
        self.tree.blockSignals(True);self.tree.clear()
        roots={k:QTreeWidgetItem(self.tree,[k.title()]) for k in ('bodies','cameras')}
        items={}
        for kind in roots:
            for row in description[kind]:
                parent=items.get(('bodies',row['parent']),roots[kind]) if kind=='bodies' else roots[kind]
                item=QTreeWidgetItem(parent,[row['name']]);item.setData(0,Qt.ItemDataRole.UserRole,(kind,row['name']))
                items[(kind,row['name'])]=item
        self.parent.clear();self.parent.addItems(['world']+[b['name'] for b in description['bodies']])
        self.target.clear();self.target.addItems([b['name'] for b in description['bodies']])
        self.tree.expandAll();self.tree.blockSignals(False)
        if items:self.tree.setCurrentItem(items.get(old,next(iter(items.values()))))

    def select(self,item,*_):
        key=item.data(0,Qt.ItemDataRole.UserRole) if item else None
        self.selected=key
        self.apply.setEnabled(bool(key));self.aim.setEnabled(bool(key and key[0]=='cameras'))
        if not key:return
        row=next(r for r in self.description[key[0]] if r['name']==key[1])
        for name,value in zip(('x','y','z','qw','qx','qy','qz'),row['position']+row['quaternion']):self.values[name].setValue(value)
        camera=key[0]=='cameras';self.parent.setEnabled(camera)
        self.apply.setText('Apply camera' if camera else 'Apply body · pause simulation')
        self.parent.setCurrentText(row['parent'])
        for name in ('width','height','fx','fy','cx','cy','near','far'):
            self.values[name].setEnabled(camera)
            if camera:self.values[name].setValue(getattr(row['config'],name))

    def submit(self):
        if not self.selected:return
        kind,name=self.selected
        p=tuple(self.values[k].value() for k in ('x','y','z'))
        q=tuple(self.values[k].value() for k in ('qw','qx','qy','qz'))
        payload=dict(name=name,position=p,rotation=q)
        if kind=='cameras':
            try:config=CameraConfig(*(self.values[k].value() for k in ('width','height','fx','fy','cx','cy','near','far')))
            except ValueError as exc:self.note.setText(str(exc));return
            payload.update(parent=self.parent.currentText(),config=config)
        self.command.emit('edit_camera' if kind=='cameras' else 'edit_body',payload)

    def aim_selected(self):
        if self.selected and self.selected[0]=='cameras':
            self.command.emit('aim_camera',dict(name=self.selected[1],target=self.target.currentText()))
