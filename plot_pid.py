import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Remplace 'COM3' par le bon port série (sur Linux/Mac : '/dev/ttyUSB0' ou '/dev/ttyACM0')
ser = serial.Serial('COM8', 9600, timeout=1)

erreurs = []
sorties_pid = []
temps = []

fig, ax = plt.subplots()
ax.set_ylim(-3, 3)  # Ajuste selon la plage de ton PID
ax.set_xlim(0, 100)  # 100 points affichés en continu
line1, = ax.plot([], [], label="Erreur")
line2, = ax.plot([], [], label="Sortie PID")
plt.legend()

def update(frame):
    global erreurs, sorties_pid, temps
    try:
        data = ser.readline().decode().strip()
        if data:
            values = data.split()
            if len(values) == 2:
                erreur = float(values[0])
                sortie_pid = float(values[1])

                erreurs.append(erreur)
                sorties_pid.append(sortie_pid)
                temps.append(len(erreurs))

                if len(erreurs) > 100:  # Garde les 100 dernières valeurs
                    erreurs.pop(0)
                    sorties_pid.pop(0)
                    temps.pop(0)

                line1.set_data(temps, erreurs)
                line2.set_data(temps, sorties_pid)
                ax.set_xlim(max(0, len(erreurs) - 100), len(erreurs))
    
    except Exception as e:
        print(f"Erreur : {e}")

    return line1, line2

ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
plt.show()
ser.close()
