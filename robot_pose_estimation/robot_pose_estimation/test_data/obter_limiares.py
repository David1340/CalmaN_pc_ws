import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog, ttk
from PIL import Image, ImageTk
import xml.etree.ElementTree as ET
import os
import time


class ColorThresholdApp:
    def __init__(self, root, video_source=0, xml_file="thresholds.xml"):
        self.root = root
        self.root.title("Seleção de Limiares de Cores (RGB) - Vídeo")

        # Fonte de vídeo
        self.cap = cv2.VideoCapture(video_source)
        # Define resolução
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Define foco manual (nem todas as câmeras suportam)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)   # 0 = manual, 1 = automático
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)       # valor em uma faixa dependente da câmera

        # Define exposição manual
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual em algumas câmeras; 3 = automático (varia conforme driver)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -8)

        # Define contraste, saturação e brilho
        self.cap.set(cv2.CAP_PROP_CONTRAST, 10)
        self.cap.set(cv2.CAP_PROP_SATURATION, 200)
        brilho = 40
        print("Definindo brilho para:", brilho)

        for _ in range(3):
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 100)
            time.sleep(0.2)
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brilho)
            time.sleep(0.2)
        
        #time.sleep(0.1)
        #self.cap.set(cv2.CAP_PROP_BRIGHTNESS, brilho)
        print("Brilho aplicado:", self.cap.get(cv2.CAP_PROP_BRIGHTNESS))
        #self.frame_teste = cv2.imread(self.image_path)

        # Lista de cores
        self.colors = ["Vermelho", "Verde", "Azul"]

        # Limiar padrão
        self.thresholds = {
            "Vermelho": {"Rmin": 0, "Rmax": 255, "Gmin": 0, "Gmax": 255, "Bmin": 0, "Bmax": 255},
            "Verde": {"Rmin": 0, "Rmax": 255, "Gmin": 0, "Gmax": 255, "Bmin": 0, "Bmax": 255},
            "Azul": {"Rmin": 0, "Rmax": 255, "Gmin": 0, "Gmax": 255, "Bmin": 0, "Bmax": 255},
        }

        self.xml_file = xml_file
        if os.path.exists(self.xml_file):
            self.load_thresholds_from_xml(self.xml_file)

        # Cor atual
        self.current_color = tk.StringVar(value=self.colors[0])

        # Frame controles
        controls_frame = tk.Frame(self.root)
        controls_frame.pack(side=tk.LEFT, padx=10, pady=10, fill="y")

        # Combobox seleção de cor
        tk.Label(controls_frame, text="Cor:").pack()
        self.color_combo = ttk.Combobox(
            controls_frame, values=self.colors, textvariable=self.current_color, state="readonly"
        )
        self.color_combo.pack(pady=5)
        self.color_combo.bind("<<ComboboxSelected>>", self.load_thresholds_to_sliders)

        # Sliders
        self.slider_vars = {}
        self.sliders = {}
        for key in ["Rmin", "Rmax", "Gmin", "Gmax", "Bmin", "Bmax"]:
            var = tk.IntVar()
            self.slider_vars[key] = var

            row = tk.Frame(controls_frame)
            row.pack(fill="x", pady=2)

            label = tk.Label(row, text=key, width=5)
            label.pack(side=tk.LEFT)

            slider = tk.Scale(
                row, from_=0, to=255, orient=tk.HORIZONTAL,
                variable=var, command=self.update_thresholds
            )
            slider.pack(side=tk.RIGHT, fill="x", expand=True)

            self.sliders[key] = slider

        # Botões salvar e carregar
        save_btn = tk.Button(controls_frame, text="Salvar Limiar", command=self.save_thresholds)
        save_btn.pack(pady=5)

        load_btn = tk.Button(controls_frame, text="Carregar Limiar", command=self.load_thresholds_dialog)
        load_btn.pack(pady=5)

        # Frame vídeo
        self.video_label = tk.Label(self.root)
        self.video_label.pack(side=tk.RIGHT, padx=10, pady=10)

        # Inicializar sliders com valores da cor atual
        self.load_thresholds_to_sliders()

        # Iniciar loop de atualização do vídeo
        self.update_video()

    def load_thresholds_from_xml(self, filepath):
        """Carregar limiares do XML"""
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
            print(f"Limiar carregado de {filepath}")
            self.load_thresholds_to_sliders()
        except Exception as e:
            print("Erro ao carregar XML:", e)

    def load_thresholds_dialog(self):
        """Abrir diálogo para carregar limiares"""
        filepath = filedialog.askopenfilename(filetypes=[("XML files", "*.xml")])
        if filepath:
            self.load_thresholds_from_xml(filepath)

    def load_thresholds_to_sliders(self, *args):
        """Carregar sliders com valores da cor atual"""
        color = self.current_color.get()
        for key, val in self.thresholds[color].items():
            self.slider_vars[key].set(val)

    def update_thresholds(self, *args):
        """Atualizar dicionário com valores dos sliders"""
        color = self.current_color.get()
        for key in self.slider_vars:
            self.thresholds[color][key] = self.slider_vars[key].get()

    def update_video(self):
        """Atualizar frame do vídeo"""
        ret, frame = self.cap.read()
        if ret:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            color = self.current_color.get()
            rmin, rmax = self.thresholds[color]["Rmin"], self.thresholds[color]["Rmax"]
            gmin, gmax = self.thresholds[color]["Gmin"], self.thresholds[color]["Gmax"]
            bmin, bmax = self.thresholds[color]["Bmin"], self.thresholds[color]["Bmax"]

            lower = (rmin, gmin, bmin)
            upper = (rmax, gmax, bmax)

            mask = cv2.inRange(frame_rgb, lower, upper)
            #Aplicar operações morfológicas
            kernel = np.ones((5, 5), np.uint8)  # kernel para operações morfológicas
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove ruído
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # preenche buracos
            result = cv2.bitwise_and(frame_rgb, frame_rgb, mask=mask)

            # Reduzir para exibição
            h, w = result.shape[:2]
            scale = 0.6
            new_w, new_h = int(w * scale), int(h * scale)
            result_small = cv2.resize(result, (new_w, new_h), interpolation=cv2.INTER_AREA)

            img_pil = Image.fromarray(result_small)
            img_tk = ImageTk.PhotoImage(img_pil)

            self.video_label.configure(image=img_tk)
            self.video_label.image = img_tk

        self.root.after(20, self.update_video)  # Atualiza a cada 20ms

    def save_thresholds(self):
        """Salvar todos os limiares em XML"""
        root = ET.Element("Thresholds")

        for color in self.colors:
            color_elem = ET.SubElement(root, color)
            for key, val in self.thresholds[color].items():
                elem = ET.SubElement(color_elem, key)
                elem.text = str(val)

        tree = ET.ElementTree(root)
        file_path = filedialog.asksaveasfilename(defaultextension=".xml",
                                                 initialfile=self.xml_file,
                                                 filetypes=[("XML files", "*.xml")])
        if file_path:
            tree.write(file_path, encoding="utf-8", xml_declaration=True)
            print(f"Limiar salvo em {file_path}")


if __name__ == "__main__":
    root = tk.Tk()
    app = ColorThresholdApp(root, video_source=1, xml_file="thresholds.xml")
    root.mainloop()
