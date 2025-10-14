import cv2

def listar_cameras(max_test=10):
    print(" Verificando cameras disponiveis...")
    available = []
    for i in range(max_test):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f" Camera {i} disponivel.")
                available.append(i)
            else:
                print(f" Camera {i} abriu mas nao retornou frame.")
            cap.release()
        else:
            print(f" Camera {i} nao disponível.")
    print("\nCameras detectadas:", available)
    return available

if __name__ == "__main__":
    listar_cameras()
