from PySide6.QtWidgets import QWidget,QVBoxLayout,QPushButton,QDialog


class PreviewPanel(QWidget):
    def __init__(self,label,view):
        super().__init__()
        self.view=view;self.label=label;self.dialog=None
        self.layout=QVBoxLayout(self)
        self.layout.addWidget(label)
        button=QPushButton('Float view');button.clicked.connect(self.detach)
        self.layout.addWidget(button);self.layout.addWidget(view,1)

    def detach(self):
        if self.dialog is not None:
            self.dialog.raise_();return
        self.dialog=QDialog(self.window())
        self.dialog.setWindowTitle(self.label.text());self.dialog.resize(800,500)
        QVBoxLayout(self.dialog).addWidget(self.view)
        self.dialog.finished.connect(self.restore)
        self.dialog.show()

    def restore(self,*_):
        dialog=self.dialog
        self.layout.addWidget(self.view,1)
        self.view.show();self.dialog=None
        if dialog is not None:dialog.deleteLater()
