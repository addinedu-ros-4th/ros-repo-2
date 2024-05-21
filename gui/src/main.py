import sys
from PyQt5.QtWidgets import QApplication, QStackedWidget
from ui_main import Ui_MainWindow
from ui_order import Ui_OrderWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    stacked_widget = QStackedWidget()

    main_window = Ui_MainWindow(stacked_widget)
    order_window = Ui_OrderWindow(stacked_widget)

    stacked_widget.addWidget(main_window)
    stacked_widget.addWidget(order_window)

    stacked_widget.setCurrentIndex(0)
    stacked_widget.show()

    sys.exit(app.exec_())
