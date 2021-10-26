# Sarduino-TTGO-T-Beam
Un décodeur de balises 406 MHz EPIRB sur un TTGO T-Beam.

# Ce travail a été réalisé par F1GHO
Vous trouverez le code source écrit sous Arduino IDE dans la version V1.1. La version V1.2 est une version compilée avec ajout d'un test interne de décodage en appuyant sur le bouton 38 du T-Beam.

# Description sommaire
Ce décodeur nécessite un récepteur externe avec sortie "discri" ainsi qu'un montage permettant la mise en forme des signaux reçus (comparable à ma version du Sarduino). Un encodeur rotatif permet une navigation dans les pages. Un "grand" écran OLED de 1,3 pouces permet une lecture aisée des infos décodées.

Le GPS intégré au T-Beam permet, une fois la balise décodée, d'avoir un cap/distance entre le T-Beam et la balise (si celle-ci comporte un GPS). Pour le reste, je vous laisse découvrir les notes de F1GHO ainsi que les images qui se suffisent à elles-même.

# Projet participatif, nous avons besoin de vos compétences !!!
Pour en faire un décodeur "parfait", il serait intéressant d'exploiter le module LoRa comme récepteur, ce qui permettrait d'en faire un décodeur portatif (dans le style des décodeurs de radiosonde pour ceux qui connaissent). Mais pour ça, je vous demande un MAXIMUM DE PARTAGE pour trouver la bonne personne qui saura trouver le moyen d'utiliser le SX1276/8 pour cette fonction.

Enfin, tout est là, dans ce dépot Github. Nul besoin de contacter F1GHO ou moi-même pour en demander plus. Néanmoins, pour toute amélioration, je vous invite à utiliser ce Github ou à défaut, de me contacter via Twitter @f4gmu pour que je puisse en faire profiter tout le monde.
