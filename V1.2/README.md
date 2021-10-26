# DECODAGE DES TRAMES BALISES 406MHz  avec module TTGO derrière un récepteur FM et mise en forme.
Attention à bien régler le comparateur pour avoir un joli signal carré et voir pas de forme bizarre (notamment au debut de trame). Voir images jointes.

Le programme que j'ai fait s'inspire d'une premiere ébauche réalisée par F4GMU.

Ce programme décode toutes les infos et celles-ci sont disponibles sur 3 sorties :
- l'ecran OLED 1,3 pouces sur le module.
- le port série USB en 115200 bauds.
- en liaison Bluetooth avec un terminal style Smartphone et une application style "Serial Bluetooth" disponible sur le Store

Après la mise en marche du module, il est prêt pour la réception d'une trame.

On peut aussi, en appuyant sur le bouton 38 du TTGO, lancer un décodage de test interne.

On peut aussi envoyer une trame par la liaison USB en 115200 bauds sans fin de ligne ni retour charriot. Ne pas mettre la trame synchro, uniquement la trame identification avec ses  15 octets du style:

```
8E92F1206334A0330CDFF70DE52163
```
   
F1GHO
