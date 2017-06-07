# Remote Diagnosis of Routers

Electronic and other devices are becoming increasingly functional, for example single devices can provide cable TV, internet and telephone services. However such functionality can come at the expense of complexity of installation and maintenance. If a device is not working to the customer’s satisfaction, then the customer typically calls support personnel during which the support personnel ask the customer to describe the status of the device and then attempts to provide resolutions to the problem. However due to the complexity of many devices, describing the status is both time consuming and error-prone, resulting in expensive, time-consuming support sessions.

The target of this project is to identify the state of a router by capturing a short video of it.The system consists of training, diagnosis and problem resolution components.

The project in this repository can work in 2 modes, training and matching. 

The input to the project is video of a router, model number(or some identifier) and locations of LED's in first frame. Under training mode, it will analyze the video and generate a template. Under matching mode it will analyze the video and match against the existing templates in the database. 
