import numpy as np, serial, time
from HersheyFonts import HersheyFonts

L1, L2 = 10.0, 10.0            # link lengths in cm
X0, Y0 = 8.0, 4.0              # text start on paper (cm)
SCALE  = 0.05
STEP   = 0.2
LINE_H = 3.0                   # cm between lines of text
MAX_W  = 20.0                  # paper width in cm

def ik(x, y):
    c2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)
    q2 = np.arccos(np.clip(c2, -1, 1))
    q1 = np.arctan2(y, x) - np.arctan2(L2*np.sin(q2), L1 + L2*np.cos(q2))
    return np.degrees(q1), np.degrees(q2)

font = HersheyFonts(); font.load_default_font()

def text_to_path(text, yoff=0.0):
    path, last = [], None
    for (a, b) in font.lines_for_text(text):
        a = (X0 + a[0]*SCALE, Y0 + yoff + a[1]*SCALE)
        b = (X0 + b[0]*SCALE, Y0 + yoff + b[1]*SCALE)
        if last is None or np.hypot(a[0]-last[0], a[1]-last[1]) > 1e-6:
            path.append((*a, 0))                        # pen up
        n = max(2, int(np.hypot(b[0]-a[0], b[1]-a[1]) / STEP))
        for t in np.linspace(0, 1, n):
            path.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), 1))
        last = b
    return path

def send(ser, path):
    for x, y, pen in path:
        q1, q2 = ik(x, -y)
        ser.write(f"{q1:.1f},{q2:.1f},{pen}\n".encode())
        ser.readline()                                  # wait for "ok"
    ser.write(b"0,0,0\n"); ser.readline()               # lift pen at the end

ser = serial.Serial('COM3', 9600, timeout=5)            # change the port
time.sleep(2)

while True:
    sentence = input("Enter text to write (or 'quit'): ").strip()
    if sentence.lower() == 'quit':
        break
    words = sentence.split()
    lines, cur = [], ""
    for w in words:                                      # simple word wrap
        test = (cur + " " + w).strip()
        if len(test) * 1.0 > MAX_W / (SCALE * 20) and cur:
            lines.append(cur); cur = w
        else:
            cur = test
    lines.append(cur)
    for i, line in enumerate(lines):
        send(ser, text_to_path(line, yoff=-i*LINE_H))
    print("Done.")

ser.close()