import csv
import matplotlib.pyplot as plt

# Listes pour stocker les données
times = []
errors = []
corrections = []
left_speeds = []
right_speeds = []

# Lecture du fichier CSV
with open('log.csv', 'r') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # sauter l'en-tête
    for row in reader:
        if len(row) < 5:
            continue
        try:
            time_ms = float(row[0])
            times.append(time_ms / 1000)  # conversion en secondes
            errors.append(float(row[1]))
            corrections.append(float(row[2]))
            left_speeds.append(int(row[3]))
            right_speeds.append(int(row[4]))
        except ValueError:
            continue  # ignore les lignes corrompues

# Affichage des graphiques
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(times, errors, label="Erreur PID", color='orange')
plt.ylabel("Erreur")
plt.grid()
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(times, corrections, label="Correction PID", color='blue')
plt.ylabel("Correction")
plt.grid()
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(times, left_speeds, label="Vitesse Gauche", color='green')
plt.plot(times, right_speeds, label="Vitesse Droite", color='red')
plt.xlabel("Temps (s)")
plt.ylabel("Vitesses Moteurs")
plt.grid()
plt.legend()

plt.tight_layout()
plt.suptitle("Analyse PID Robot suiveur de ligne", fontsize=16, y=1.02)
plt.show()
