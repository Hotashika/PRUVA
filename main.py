import sys
from PyQt5 import QtCore, QtWidgets
from src.presentation.controller import NjordAnaEkran

# Hata giderici yama
for attr, val in [('Dec', 1), ('Flat', 0), 
                  ('QDialogButtonBox::StandardButton::Cancel', QtWidgets.QDialogButtonBox.Cancel),
                  ('QDialogButtonBox::StandardButton::Ok', QtWidgets.QDialogButtonBox.Ok)]:
    if not hasattr(QtCore.Qt, attr): setattr(QtCore.Qt, attr, val)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    ana = NjordAnaEkran()
    ana.show()
    sys.exit(app.exec_())