import cv2
import tkinter as tk
from tkinter import filedialog, ttk
from PIL import Image, ImageTk
import xml.etree.ElementTree as ET


class ColorThresholdApp:
    def __init__(self, root, image_path):
        self.root = root
        self.root.title("Seleção de Limiares de Cores (RGB)")

        # Carregar imagem
        self.image = cv2.imread(image_path)
        self.image_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        # Armazena limiares de cada cor
        self.colors = ["Vermelho", "Verde", "Azul"]
        self.thresholds = {
            "Vermelho": {"Rmin": 0, "Rmax": 255, "Gmin": 0, "Gmax": 255, "Bmin": 0, "Bmax": 255},
            "Verde": {"Rmin": 0, "Rmax": 255, "Gmin": 0, "Gmax": 255, "Bmin": 0, "Bmax": 255},
            "Azul": {"Rmin": 0, "Rmax": 255, "Gmin": 0, "Gmax": 255, "Bmin": 0, "Bmax": 255},
        }

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

        # Sliders (variáveis dinâmicas)
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
                variable=var, command=self.update_image
            )
            slider.pack(side=tk.RIGHT, fill="x", expand=True)

            self.sliders[key] = slider

        # Botões salvar e carregar
        save_btn = tk.Button(controls_frame, text="Salvar Limiar", command=self.save_thresholds)
        save_btn.pack(pady=5)

        load_btn = tk.Button(controls_frame, text="Carregar Limiar", command=self.load_thresholds_dialog)
        load_btn.pack(pady=5)

        # Frame imagem
        self.image_label = tk.Label(self.root)
        self.image_label.pack(side=tk.RIGHT, padx=10, pady=10)

        # Carrega sliders da cor inicial
        self.load_thresholds_to_sliders()

    def load_thresholds_to_sliders(self, *args):
        """Carregar sliders com valores da cor atual"""
        color = self.current_color.get()
        for key, val in self.thresholds[color].items():
            self.slider_vars[key].set(val)
        self.update_image()

    def update_image(self, *args):
        """Atualizar visualização"""
        color = self.current_color.get()

        # Atualizar dicionário da cor atual
        for key in self.slider_vars:
            self.thresholds[color][key] = self.slider_vars[key].get()

        rmin, rmax = self.thresholds[color]["Rmin"], self.thresholds[color]["Rmax"]
        gmin, gmax = self.thresholds[color]["Gmin"], self.thresholds[color]["Gmax"]
        bmin, bmax = self.thresholds[color]["Bmin"], self.thresholds[color]["Bmax"]

        lower = (rmin, gmin, bmin)
        upper = (rmax, gmax, bmax)

        mask = cv2.inRange(self.image_rgb, lower, upper)

        # Aplicar a máscara na imagem colorida
        result = cv2.bitwise_and(self.image_rgb, self.image_rgb, mask=mask)

        # --- Reduzir tamanho da imagem para exibição ---
        h, w = result.shape[:2]
        scale = 0.6   # 60% do tamanho original
        new_w, new_h = int(w * scale), int(h * scale)
        result_small = cv2.resize(result, (new_w, new_h), interpolation=cv2.INTER_AREA)

        img_pil = Image.fromarray(result_small)
        img_tk = ImageTk.PhotoImage(img_pil)

        self.image_label.configure(image=img_tk)
        self.image_label.image = img_tk

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
                                                 filetypes=[("XML files", "*.xml")])
        if file_path:
            tree.write(file_path, encoding="utf-8", xml_declaration=True)
            print(f"Limiar salvo em {file_path}")

    def load_thresholds_dialog(self):
        """Abrir diálogo para carregar limiares de um XML"""
        file_path = filedialog.askopenfilename(filetypes=[("XML files", "*.xml")])
        if file_path:
            try:
                tree = ET.parse(file_path)
                root = tree.getroot()

                for color in self.colors:
                    color_elem = root.find(color)
                    if color_elem is not None:
                        for key in self.thresholds[color]:
                            elem = color_elem.find(key)
                            if elem is not None:
                                self.thresholds[color][key] = int(elem.text)

                print(f"Limiar carregado de {file_path}")
                self.load_thresholds_to_sliders()
            except Exception as e:
                print("Erro ao carregar XML:", e)


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Uso: python script.py <imagem>")
        sys.exit(1)

    root = tk.Tk()
    app = ColorThresholdApp(root, sys.argv[1])
    root.mainloop()
