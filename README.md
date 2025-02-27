# Nicla Vision Waste Classification 

This project uses **Edge Impulse** on **Nicla Vision** to classify waste as **Recyclable**, **Non-Recyclable**, and detect anomalies (trash on the ground). The system lights up different LEDs to indicate the classification.

## 📌 Features
- Machine learning-based waste classification
- Uses **Nicla Vision** for real-time inference
- LEDs indicate classification results:
  - 🟢 **Green**: Recyclable
  - 🔴 **Red**: Non-recyclable
  - 🟡 **Yellow**: Anomaly (Trash on the ground)

## 📂 Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/Hashim11Mub/nicla-waste-classification.git


Install the Edge Impulse Library

	1.	Inside the cloned folder, ZIP the nicla-waste-classification directory.
	2.	Open Arduino IDE and navigate to:
	•	Sketch ➝ Include Library ➝ Add .ZIP Library…
	3.	Select the zipped nicla-waste-classification folder and import it.

Run the Example Code

	1.	In Arduino IDE, go to:
	•	File ➝ Examples ➝ _lk_inferencing ➝ nicla_vision_camera.ino
	2.	Open the file and upload it to the Nicla Vision board.
	3.	The model will be uploaded and start running inference.
