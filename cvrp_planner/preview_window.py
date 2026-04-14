import os
import sys

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QScrollArea


class PreviewWindow(QMainWindow):
    def __init__(self, image_path):
        super().__init__()
        self.image_path = image_path
        self.original_pixmap = QPixmap(image_path)

        self.setWindowTitle('CVRP Strategy Preview')
        self.resize(1200, 900)

        self.scroll_area = QScrollArea(self)
        self.scroll_area.setWidgetResizable(True)
        self.setCentralWidget(self.scroll_area)

        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        self.scroll_area.setWidget(self.label)

        if self.original_pixmap.isNull():
            self.label.setText(f'预览图加载失败:\n{image_path}')
        else:
            self._update_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self.original_pixmap.isNull():
            self._update_pixmap()

    def _update_pixmap(self):
        viewport_size = self.scroll_area.viewport().size()
        scaled = self.original_pixmap.scaled(
            viewport_size,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.label.setPixmap(scaled)


def main():
    if len(sys.argv) < 2:
        print('Usage: python -m cvrp_planner.preview_window <image_path>')
        return 1

    image_path = os.path.abspath(sys.argv[1])
    app = QApplication(sys.argv)
    window = PreviewWindow(image_path)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
