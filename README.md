Les exigences pour le projet CEG 4566 sont présentés ci-dessous. Pour plus de détails sur l'implémentation, référez-vous à la section **Wiki**.

Exigences Fonctionnelles
- Détection des objets : Le système doit détecter les objets présents dans son environnement dans la plage de détection définie.
- Alarme de proximité : Lorsqu’un objet entre dans une zone de proximité définie par l’utilisateur, une alarme sonore doit se déclencher pour signaler la présence et potentiellement dissuader l’intrusion.
- Détection en temps réel : Le système doit analyser son environnement et signaler les détections sans délai notable.
- Balayage continu : Le capteur de détection doit scanner en permanence son environnement dans sa plage de détection pour identifier toute présence d’objet.
- Transmission des données : Les données collectées doivent être envoyées à une application web avec de la communication wireless.
- Visualisation des détections : L’application web doit présenter les données de détection et leur proximité pour l’utilisateur.
- Gestion de l’énergie : Lorsqu’aucun mouvement n’est détecté, le système doit passer en mode basse consommation pour préserver l’énergie.

Exigences Non-Fonctionnelles
- Fiabilité : Les données sur les détections et leurs proximités doivent être juste et assez précis.
- Portabilité : Le système doit être portable pour être utilisable dans différents scénarios.
- Latence faible : Le délai entre une détection et la visualisation sur l’application web ne devrait pas dépasser 1 seconde.
