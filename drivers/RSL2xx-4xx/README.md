# RSL200 / RSL400

Drivery pro LIDARy RSL2xx a RSL4xx jsou oficiálně poskytovány společností Leuze a lze je stáhnout na stránkách příslušného senzoru (například [zde](https://www.leuze.com/en-int/rsl410-s-cu411-rs4/53800245)) v sekci *Downloads*.

Jsou dostupné ve verzích pro ROS1 i ROS2 a doprovází je podrobný návod v souboru *readme*. Klíčové je, že driver zpracovává syrová navigační UDP data ze senzoru a převádí je do ROS zprávy typu ```LaserScan```. Tuto zprávu publikuje vždy po přijetí kompletního skenu.
#

Drivers for the RSL2xx and RSL4xx LIDARs are officially released by Leuze and can be downloaded from the respective sensor’s page (e.g., [here](https://www.leuze.com/en-int/rsl410-s-cu411-rs4/53800245)) in the Downloads section.

They are available for both ROS 1 and ROS 2, with detailed instructions provided in the accompanying *readme* file. Importantly, the driver processes raw UDP navigation data from the sensor and converts it into a ```LaserScan``` message compatible with ROS, publishing it after receiving each complete scan.