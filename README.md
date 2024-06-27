
![AUV](https://github.com/NarjesDavari/Underwater_Navigation/blob/main/AUV.png)

![trajec](https://github.com/NarjesDavari/Underwater_Navigation/blob/main/Vessel.png)

# Underwater_Navigation
Algorithms for underwater integrated navigation system

The designed real navigation system is composed of a high-rate strapdown inertial navigation system along with low-rate auxiliary sensors with different sampling rates. The auxiliary sensors consist of a global positioning system (GPS), a Doppler velocity log (DVL), a depthmeter, and an inclinometer.
To integrate SDINS with data from auxiliary sensors, different algorithms are developed such as, variational Bayesian Kalman filter, Asynchronous Adaptive Direct Kalman [1-4].

Also, In this repository, there is method for on-line outlier detection based NN [5].

We have some related published works:

1- An asynchronous adaptive direct Kalman filter algorithm to improve underwater navigation system performance
N Davari, A Gholami. IEEE Sensors Journal 17 (4), 1061-1068.

2- "Asynchronous direct Kalman filtering approach for underwater integrated navigation system," M Shabani, A Gholami, N Davari
Nonlinear Dynamics 80, 71-85

3- "Variational Bayesian adaptive Kalman filter for asynchronous multirate multi-sensor integrated navigation system," N Davari, A Gholami. Ocean Engineering 174, 108-116

4- "Multirate adaptive Kalman filter for marine integrated navigation system," N Davari, A Gholami, M Shabani. The Journal of Navigation 70 (3), 628-647

5- "Real-time outlier detection applied to a Doppler velocity log sensor based on hybrid autoencoder and recurrent neural network"
N Davari, AP Aguiar. IEEE Journal of Oceanic Engineering 46 (4), 1288-1301
