DECODAGE DES TRAMES BALISES 406MHz  avec module TTGO derriere un récepteur FM et mise en forme
attention à bien régler le comparateur pour avoir un joli signal carré et voir pas de forme bizarre (notamment au debut de trame)
voir images jointes

le programme que j'ai fait sinspire d'une premiere ébauche réalisée par F4GMU.

Ce programme decode toutes les infos et celle-ci sont disponibles sur 3 sorties:
- l'ecran OLED 1.3 pouces sur le module 
- le port série USB en 115200 bauds
- en liaison Bluetooth avec un terminal style Smartphone et une appli style "Serial Bluetooth" dispo sur le Store


après la mise en marche du module il est prêt pour la réception d'une trame.

on peut aussi en appuyant sur le "bouton 38" lancer un decodage de test interne.

on peut aussi envoyer une trame par la liaison USB en 115200 bauds sans fin de ligne ni retour charriot.
ne pas mettre la trame synchro, uniquement la trame identification avec ses  15 octets du style:
   8E92F1206334A0330CDFF70DE52163
   
   