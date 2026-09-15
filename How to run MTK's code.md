##### Frontend Run
vscode cmd prompt မှာ

cd frontend
npm run dev

http://localhost:5173/ ကို browser မှာဖွင့်

##### Backend Run
powershell နဲ့ ဒါတွေတစ်ကြောင်းစီ  run

cd backend
$env:MQTT_ENABLED = "true"  
$env:MQTT_HOST = "localhost"  
$env:MQTT_PORT = "1883"  
python app.py

"only for mac"
MQTT_ENABLED=true MQTT_HOST=localhost WAITRESS_THREADS=20 python app.py

##### Camera Vision Run
vscode cmd prompt မှာ

cd backend
waitress-serve --host=0.0.0.0 --port=8001 --threads=20 --call object_detection.object_detection_app:create_object_detection_app


`mosquitto_pub -h localhost -t agv/R1/node -m '{"node_id":"B7C53F5"}'

MQTT works ဖို့ Firewall မှာ inbound port လုပ်ရမယ်

![[How to Create an Inbound Rule for Port 1883]]

- Windows Defender Firewall with Advanced Security ကို ဖွင့်
- Inbound Rules ကို ရွေး (ဘယ်ဘက်မှာရှိ)
- New Rule ကို နှိပ် (ညာဘက်မှာရှိ)
- Port ကိုရွေး, Next နှိပ်
- TCP ကိုရွေး
- Specific Local Ports: မှာ 1883 လို ရေး, Next နှိပ်
- Allow the connection ကိုရွေး, Next နှိပ်
- သုံးခုအမှန်ခြစ်ဖြစ်ပြီးသားတွေ့လိမ့်မယ် Next နှိပ်
- Name, Description ထည့်, Finish နှိပ်

##### Mosquito config file ပြင်ရမယ်

- Open the folder where Mosquitto is installed (usually `C:\Program Files\mosquitto`).
    
- Find the file named `mosquitto.conf`.
    
- Open it using Notepad (you may need to right-click Notepad and select "Run as Administrator" first so you have permission to save changes).
    
- Scroll to the bottom of the file and paste these two exact lines:    
    ```
    listener 1883 0.0.0.0
    allow_anonymous true
    ```
- Save the file.
    
- **Crucial:** You must restart the Mosquitto service for this to take effect. Press the **Windows Key**, type `Services`, press **Enter**, find **Mosquitto Broker** in the list, right-click it, and select **Restart**.

[[_Thesis]]


Git
======
How to create new branch?
git switch -c <branch-name>
This creates the branch and switches to it immediately.

How to push the new branch to GitHub?
git push -u origin <branch-name>

How to merge with main branch?
git checkout main
git merge <branch_name>
git push origin main