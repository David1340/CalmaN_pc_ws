import sys
import cv2
import numpy as np
import xml.etree.ElementTree as ET
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QComboBox, QSlider, QPushButton,
    QHBoxLayout, QVBoxLayout, QTabWidget, QFileDialog
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import os
import time


class CameraTab(QWidget):
    def __init__(self, video_source=0, xml_file="camera_config.xml"):
        super().__init__()
        self.video_source = video_source
        self.xml_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "test_data", xml_file)
        self.cap = cv2.VideoCapture(video_source, cv2.CAP_DSHOW)

        # Configurações padrão
        self.properties = {
                "Brilho": (cv2.CAP_PROP_BRIGHTNESS, 0, 255, 60),
                "Contraste": (cv2.CAP_PROP_CONTRAST, 0, 255, 50),
                "Saturação": (cv2.CAP_PROP_SATURATION, 0, 255, 200),
                "Exposição": (cv2.CAP_PROP_EXPOSURE, -10, 0, -2),
                "Foco": (cv2.CAP_PROP_FOCUS, 0, 255, 0),
            }

        # Carregar valores do XML se existir
        self.sliders = {}
        self.value_labels = {}
        if os.path.exists(self.xml_file):
            print("Carregando configurações de:", self.xml_file)
            self.load_properties_from_xml(self.xml_file)

        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(60)

    def init_ui(self):
        layout = QHBoxLayout()
        control_layout = QVBoxLayout()
        video_layout = QVBoxLayout()

        # Seleção de câmera
        self.camera_combo = QComboBox()
        self.camera_combo.addItems([str(i) for i in range(4)])
        self.camera_combo.setCurrentIndex(self.video_source)
        self.camera_combo.currentIndexChanged.connect(self.change_camera)
        control_layout.addWidget(QLabel("Selecionar câmera:"))
        control_layout.addWidget(self.camera_combo)

        # Sliders de propriedades
        for prop, (cap_id, min_val, max_val, default) in self.properties.items():
            label = QLabel(f"{prop}: {int(self.cap.get(cap_id))}")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            slider.setValue(int(self.cap.get(cap_id)))
            slider.valueChanged.connect(lambda val, cid=cap_id, l=label: self.update_property(cid, val, l))
            control_layout.addWidget(label)
            control_layout.addWidget(slider)
            self.sliders[prop] = slider
            self.value_labels[prop] = label
        
        # Botões salvar/carregar XML
        save_btn = QPushButton("Salvar Configuração")
        save_btn.clicked.connect(self.save_properties)
        load_btn = QPushButton("Carregar Configuração")
        load_btn.clicked.connect(self.load_properties_dialog)
        control_layout.addWidget(save_btn)
        control_layout.addWidget(load_btn)

        # Label de vídeo
        self.video_label = QLabel()
        video_layout.addWidget(self.video_label)

        # Aplicar valores iniciais ao hardware
        for prop, (cap_id, *_rest) in self.properties.items():
            self.cap.set(cap_id, self.sliders[prop].value())

        layout.addLayout(control_layout)
        layout.addLayout(video_layout)
        self.setLayout(layout)

    def update_property(self, prop_id, val, label):
        self.cap.set(prop_id, val)
        label.setText(f"{label.text().split(':')[0]}: {val}")

    def change_camera(self, index):
        self.cap.release()
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        self.load_properties_from_xml(self.xml_file)

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            scale = 0.6  # 60% do tamanho original, ajuste conforme desejar
            frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimg))
    # ------------------------------
    # XML
    # ------------------------------
    def save_properties(self):
        root = ET.Element("CameraProperties")
        for prop, (cap_id, *_rest) in self.properties.items():
            elem = ET.SubElement(root, prop)
            elem.text = str(self.sliders[prop].value())
        file_path, _ = QFileDialog.getSaveFileName(self, "Salvar XML",  os.path.join(os.path.dirname(os.path.realpath(__file__)), "test_data", "camera_config.xml"), "XML Files (*.xml)")
        if file_path:
            ET.ElementTree(root).write(file_path, encoding="utf-8", xml_declaration=True)
            print(f"Configurações salvas em {file_path}")

    def load_properties_from_xml(self, filepath):
        # Define resolução
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Desativa auto exposição
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Desativa auto foco
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)  # Define foco manual para 0
        try:
            tree = ET.parse(filepath)
            root = tree.getroot()
            for prop, (cap_id, min_val, max_val, default) in self.properties.items():
                elem = root.find(prop)
                if elem is not None:
                    val = int(elem.text)
                    self.cap.set(cap_id, val)
                    time.sleep(0.1)  # aguarda aplicação
                    self.cap.set(cap_id, default)
                    time.sleep(0.1)
                    self.cap.set(cap_id, val)
        except Exception as e:
            print("Erro ao carregar XML:", e)

    def load_properties_dialog(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Abrir XML", os.path.join(os.path.dirname(os.path.realpath(__file__)), "test_data"), "XML Files (*.xml)")
        if file_path:
            self.load_properties_from_xml(file_path)


class ThresholdTab(QWidget):
    def __init__(self, video_source=0, xml_file="limiares.xml"):
        super().__init__()
        self.video_source = video_source
        self.cap = cv2.VideoCapture(video_source, cv2.CAP_DSHOW)
        self.colors = ["Vermelho", "Verde", "Azul"]
        self.thresholds = {
            c: {k: 0 if "min" in k.lower() else 255 for k in ["Rmin","Rmax","Gmin","Gmax","Bmin","Bmax"]}
            for c in self.colors
        }
        self.xml_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "test_data", xml_file)
        if os.path.exists(self.xml_file):
            print("Carregando configurações de:", self.xml_file)
            self.load_thresholds_from_xml(self.xml_file)

        self.current_color = self.colors[0]
        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(60)


    def init_ui(self):
        layout = QHBoxLayout()
        control_layout = QVBoxLayout()
        video_layout = QVBoxLayout()

        # Seleção de câmera
        self.camera_combo = QComboBox()
        self.camera_combo.addItems([str(i) for i in range(4)])
        self.camera_combo.setCurrentIndex(self.video_source)
        self.camera_combo.currentIndexChanged.connect(self.change_camera)
        control_layout.addWidget(QLabel("Selecionar câmera:"))
        control_layout.addWidget(self.camera_combo)

        # Combobox de cores
        self.color_combo = QComboBox()
        self.color_combo.addItems(self.colors)
        self.color_combo.currentTextChanged.connect(self.load_thresholds_to_sliders)
        control_layout.addWidget(QLabel("Cor:"))
        control_layout.addWidget(self.color_combo)

        # Sliders
        self.sliders = {}
        for key in ["Rmin","Rmax","Gmin","Gmax","Bmin","Bmax"]:
            label = QLabel(f"{key}: {self.thresholds[self.current_color][key]}")
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(255)
            slider.setValue(self.thresholds[self.current_color][key])
            slider.valueChanged.connect(lambda val, k=key, l=label: self.update_threshold(k, val, l))
            control_layout.addWidget(label)
            control_layout.addWidget(slider)
            self.sliders[key] = slider

        # Botões salvar/carregar
        save_btn = QPushButton("Salvar Limiar")
        save_btn.clicked.connect(self.save_thresholds)
        load_btn = QPushButton("Carregar Limiar")
        load_btn.clicked.connect(self.load_thresholds_dialog)
        control_layout.addWidget(save_btn)
        control_layout.addWidget(load_btn)

        # Label de vídeo
        self.video_label = QLabel()
        video_layout.addWidget(self.video_label)

        layout.addLayout(control_layout)
        layout.addLayout(video_layout)
        self.setLayout(layout)

    def change_camera(self, index):
        self.cap.release()
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)

    def load_thresholds_from_xml(self, filepath):
        try:
            tree = ET.parse(filepath)
            root = tree.getroot()
            for color in self.colors:
                color_elem = root.find(color)
                if color_elem is not None:
                    for key in self.thresholds[color]:
                        elem = color_elem.find(key)
                        if elem is not None:
                            self.thresholds[color][key] = int(elem.text)
        except Exception as e:
            print("Erro ao carregar XML:", e)

    def load_thresholds_dialog(self):
        filepath, _ = QFileDialog.getOpenFileName(self, "Abrir XML", os.path.join(os.path.dirname(os.path.realpath(__file__)), "test_data"), "XML Files (*.xml)")
        if filepath:
            self.load_thresholds_from_xml(filepath)
            self.load_thresholds_to_sliders()

    def load_thresholds_to_sliders(self):
        self.current_color = self.color_combo.currentText()
        for key, val in self.thresholds[self.current_color].items():
            self.sliders[key].blockSignals(True)
            self.sliders[key].setValue(val)
            self.sliders[key].blockSignals(False)

    def update_threshold(self, key, val, label):
        self.thresholds[self.current_color][key] = val
        label.setText(f"{key}: {val}")

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            c = self.current_color
            lower = (self.thresholds[c]["Rmin"], self.thresholds[c]["Gmin"], self.thresholds[c]["Bmin"])
            upper = (self.thresholds[c]["Rmax"], self.thresholds[c]["Gmax"], self.thresholds[c]["Bmax"])
            mask = cv2.inRange(frame, lower, upper)
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            scale = 0.6  # 60% do tamanho original, ajuste conforme desejar
            result = cv2.resize(result, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            h, w, ch = result.shape
            bytes_per_line = ch*w
            qimg = QImage(result.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimg))

    def save_thresholds(self):
        root = ET.Element("Thresholds")
        for color in self.colors:
            color_elem = ET.SubElement(root, color)
            for key, val in self.thresholds[color].items():
                elem = ET.SubElement(color_elem, key)
                elem.text = str(val)
        filepath, _ = QFileDialog.getSaveFileName(self, "Salvar XML",  os.path.join(os.path.dirname(os.path.realpath(__file__)), "test_data", "limiares.xml"), "XML Files (*.xml)")
        if filepath:
            ET.ElementTree(root).write(filepath, encoding="utf-8", xml_declaration=True)


def main():
    app = QApplication(sys.argv)
    main_win = QTabWidget()
    main_win.setWindowTitle("Controle de Câmera e Limiar de Cores")

    main_win.addTab(CameraTab(video_source=0), "Câmera")
    main_win.addTab(ThresholdTab(video_source=0), "Limiares")

    main_win.resize(1200, 600)
    main_win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

