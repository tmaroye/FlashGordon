![FlashGordon] (https://github.com/tmaroye/FlashGordon/blob/master/Photos/FlashGordon.png)

# FlashGordon
Arduino Mini Suno Robot

## Introduction
Bonjour,

Mon projet est de réaliser un robot sumo de comptétition dans la catégorie Mini Sumo.

## Matériel utilisé
Voici la liste des éléments qui constituent à ce jour mon robot.

### La carte contrôleur :
J'ai choisi une carte BLUNO parcequ'elle intègre une interphace Bluetooth 4.0 et que son bus I2C n'est pas jumelé avec les ports analogiques du contôleur. Cela me fait gagner deux entrées analogiques.

![BLUNO] (https://github.com/tmaroye/FlashGordon/blob/master/Photos/bluno.jpg)

### La motorisation
Comme je ne savais vraiment pas par ou commencer, j'ai décidé d'utiliser un chassis tout fait.
Il y en a deux que je touve bien :

-Le Zuno de Pololu : <https://www.pololu.com/product/2506>

![Zuno](https://mcuoneclipse.files.wordpress.com/2014/07/new-zumo-robot.png)

-Le Cobra de Fingertech: <http://www.fingertechrobotics.com/proddetail.php?prod=ft-kit-cobra-chassis>

![Cobra] (http://www.fingertechrobotics.com/prodimages/kits/Rhino_top.png)

J'ai choisi le **Cobra en version 50:1** parce que plus rapide.

Et pour rêver : <http://www.dfrobot.com/index.php?route=product/product&path=37&product_id=1315#.VlF4Vd8vdhE>


### Centrale inertielle
Pour la corretion de direction et la détection de choc, j'utilise un GY-88.

![GY-88] (https://github.com/tmaroye/FlashGordon/blob/master/Photos/GY88.jpg)

<https://hackspark.fr/fr/10dof-imu-gy-88-barometer-gyroscope-accelerometer-magnetometer.html>


### Télécommande IR
Pour communiquer mes commandes au robot j'utilise une télécommande IR.

![Télécommande IR] (https://github.com/tmaroye/FlashGordon/blob/master/Photos/bluno.jpg)

<http://www.gotronic.fr/art-kit-ir-pour-arduino-dfr0107-19322.htm>


### Affichage : matrice 8x8
Une mattrice 8x8 est utilisée pour connaitre l'état du robot.

![Matrice 8x8] (https://github.com/tmaroye/FlashGordon/blob/master/Photos/matrice.jpg)

<http://www.miniinthebox.com/fr/module-d-affichage-max7219-dot-matrix-module-module-de-microcontroleur-du-module-de-commande-arduino_p639090.html>


### Batterie LIPO
La batterie est une LIPO 3S de 500 mAH 35C.




