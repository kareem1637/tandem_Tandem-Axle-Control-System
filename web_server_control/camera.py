from flask import Flask, render_template, request, Response
import requests

app = Flask(__name__)

# ESP32 IP addresses
ESP1_IP = 'http://192.168.4.1'
ESP2_IP = 'http://192.168.4.4'
ESP32_CAM_IP = "http://192.168.4.5"

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/motor', methods=['GET'])
def motor():
    direction = request.args.get('direction')
    esp = request.args.get('esp')
    if direction in ['forward', 'backward'] and esp in ['1', '2']:
        esp_ip = ESP1_IP if esp == '1' else ESP2_IP
        response = requests.get(f'{esp_ip}/motor', params={'direction': direction})
        return response.text, response.status_code
    else:
        return "Invalid request parameters", 400

@app.route('/servo', methods=['GET'])
def servo():
    direction = request.args.get('direction')
    esp = request.args.get('esp')
    if direction in ['left', 'right'] and esp in ['1', '2']:
        esp_ip = ESP1_IP if esp == '1' else ESP2_IP
        response = requests.get(f'{esp_ip}/servo', params={'direction': direction})
        return response.text, response.status_code
    else:
        return "Invalid request parameters", 400

@app.route('/lock', methods=['GET'])
def lock():
    esp = request.args.get('esp')
    if esp in ['1', '2']:
        esp_ip = ESP1_IP if esp == '1' else ESP2_IP
        response = requests.get(f'{esp_ip}/lock')
        return response.text, response.status_code
    else:
        return "Invalid request parameters", 400

@app.route('/unlock', methods=['GET'])
def unlock():
    esp = request.args.get('esp')
    if esp in ['1', '2']:
        esp_ip = ESP1_IP if esp == '1' else ESP2_IP
        response = requests.get(f'{esp_ip}/unlock')
        return response.text, response.status_code
    else:
        return "Invalid request parameters", 400

@app.route('/tare', methods=['GET'])
def tare():
    esp = request.args.get('esp')
    type_ = request.args.get('type')
    if esp in ['1', '2'] and type_ in ['drive', 'steer', 'tandem']:
        esp_ip = ESP1_IP if esp == '1' else ESP2_IP
        response = requests.get(f'{esp_ip}/tare', params={'type': type_})
        return response.text, response.status_code
    else:
        return "Invalid request parameters", 400

@app.route('/calibrate', methods=['GET'])
def calibrate():
    esp = request.args.get('esp')
    type_ = request.args.get('type')
    weight = request.args.get('weight')
    if esp in ['1', '2'] and type_ in ['drive', 'steer', 'tandem'] and weight:
        esp_ip = ESP1_IP if esp == '1' else ESP2_IP
        response = requests.get(f'{esp_ip}/calibrate', params={'type': type_, 'weight': weight})
        return response.text, response.status_code
    else:
        return "Invalid request parameters", 400

@app.route('/set_ip', methods=['POST'])
def set_ip():
    ip_address = request.form['ip_address']
    response1 = requests.post(f'{ESP1_IP}/set_ip', data={'ip_address': ip_address})
    response2 = requests.post(f'{ESP2_IP}/set_ip', data={'ip_address': ip_address})
    if response1.status_code == 200 and response2.status_code == 200:
        return 'IP address updated successfully!', 200
    else:
        return 'Failed to update IP address!', 500

def gen_frames():
    url = f"{ESP32_CAM_IP}/"  # Assuming this URL provides a valid MJPEG stream
    while True:
        try:
            response = requests.get(url, stream=True)
            response.raise_for_status()
            for chunk in response.iter_content(chunk_size=1024):
                if chunk:
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + chunk + b'\r\n')
        except requests.RequestException as e:
            print(f"Request Exception: {e}")
            break
        except Exception as ex:
            print(f"Exception: {ex}")
            break
        finally:
            response.close()
            del response

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=8000)

