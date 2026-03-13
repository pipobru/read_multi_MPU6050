# [Titre]

## À propos

Permet de lire 5 capteurs MPU6050, le plus rapidement possible pour que les données soient cohérentes.
Les capteurs sont interfacés avec un BUS 12C via un multiplexeur TCA 9548A

## Table des matières

- 🪧 [À propos](#à-propos)
- 📦 [Prérequis](#prérequis)
- 🚀 [Installation](#installation)
- 🛠️ [Utilisation](#utilisation)
- 🏗️ [Construit avec](#construit-avec)
- 📚 [Documentation](#documentation)

## Prérequis

Il faut disposer d'un raspberry (ou équivalent avec un compilateur g++). Disposer de platines de développement électronique pour pouvoir connecter le multiplexeur et les capteurs MPU6050.

## Installation

Tout est rassemblé dans un seul fichier source. (Il est possible de faire un .h pour plus de lisibilité)
Configuration à faire sur le raspberry :
  Sudo apt update
  Sudo apt upgrade
  sudo raspi-config ==> Activer l’I2C
  sudo reboot
  sudo apt-get install i2c-tools libi2c-dev
	
## Utilisation
Compilation et lancement
	g++ -o readmpu6050 ./readmpu6050.c
	./readmpu6050

Commandes pour tester le bus I2C en mode console : 
	sudo i2cdetect -y 1 ==> Scan du bus I2C 1
	i2cset -y 1 0x70 0x01 ==> Activer le port 0 du multiplexeur 
	i2cget -y 1 0x70 ==> Lire quel port est actif sur le multiplexeur

	Test du multiplexer:
		i2cset -y 1 0x70 0x00 ==> On désactive tous les ports du multiplexeur
		i2cdetect -y 1
	 	    0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
		00:                         -- -- -- -- -- -- -- --
		10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		70: 70 -- -- -- -- -- -- --
		==> On ne voit plus que le multiplexeur car il ne replay plus rien

		i2cset -y 1 0x70 0x01 ==> On active le port 1 sur le multiplexeur 
		i2cdetect -y 1
		     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
		00:                         -- -- -- -- -- -- -- --
		10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
		60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
		70: 70 -- -- -- -- -- -- --
		==> On voit bien apparaitre le capteur à l’adresse 0x68
	

## Construit avec

Langage de programation c++
IDE de developpement Visual Studio Code

## Documentation
	https://passionelectronique.fr/tutorial-tca9548a/
	https://www.ti.com/lit/ds/symlink/tca9548a.pdf
	https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf#%5B%7B%22num%22%3A68%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C62%2C692%2C0%5D

Voir le fichier [LICENSE](./LICENSE.md) du dépôt.


