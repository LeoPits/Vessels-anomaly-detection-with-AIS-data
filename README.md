# Vessels anomaly detection with AIS data
## Introduction
We takes inspiration from  TREAD* (Traffic Route Extraction and Anomaly Detection)[1], that
learns a statistical model for maritime traffic from AIS data in an unsupervised way
## Dataset
National Oceanic and Atmospheric Administration: https://marinecadastre.gov/accessais

The information contained in the AIS data (generally) are of dynamic and static type
![dataset](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/61b3d2a2-d571-4d36-bea3-f4ad43aa7a58)

Area of interest: **Hawaii**

A bounding box is selected and it corresponds to the (chosen) specific area under surveillance
![image](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/a52f0ff1-dbca-4253-9bdf-86fc5f8004c2) 

## Waypoints identification
Waypoints (WPs): identify either stationary points (ports, offshore platforms, etc.), entry points and exit points.

WPs  identification is based on DBSCAN (Density-Based Spatial Clustering of Applications with Noise) method

Note: The hyperparameters of DBSCAN are tuned manually.

![image](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/a9aaef02-f73f-4803-98a2-aba101fd8613)

## Waypoints identification
Once the waypoints are learned, a route can be built by clustering the extracted vessel flows, which connect:

* Two ports
* Entry point and port 
* Port and Exit point 
* Entry point and Exit point (i.e., transit routes)
![image](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/4fb8a983-2791-4603-b7d3-30a3011d17f3)

## PoL learning (work in progress)
It is necessary to remove outliers from the routes to identify normal traffic.
In TREAD, it is used the stochastic method **Kernel Density Estimation (KDE)**.

![image](https://github.com/LeoPits/AIS_anomaly_detection/assets/19689590/c95ea577-764b-41f3-aef4-9402470dcaad)

## References
[1] *Pallotta, Giuliana, Vespe, Michele and Bryan, Karna. "Vessel pattern knowledge
discovery from AIS data: A framework for anomaly detection and route prediction."
Entropy 15.6 (2013): 2218-2245.
