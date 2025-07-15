# connect_graph 수정 코드
import os
import json
import paho.mqtt.client as mqtt
import time
import base64
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mplcursors
import threading
from datetime import datetime
from matplotlib.widgets import Button

Hostname = "mqtt.thingstream.io"
DeviceID = "device:401bd3e2-fb92-42d9-97dc-f990d6467d94"
Username = "MX5CMIDPC8OQFSBOJXO6"
Password = "oga/JJJ3E1EWT+xvUF42kZqwEXrlZiDGP9FwxUca"

dataset = []
last_save_time = time.time()

import os
import json
import paho.mqtt.client as mqtt
import time
import base64
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mplcursors
import threading
from datetime import datetime
from matplotlib.widgets import Button

Hostname = "mqtt.thingstream.io"
DeviceID = "device:401bd3e2-fb92-42d9-97dc-f990d6467d94"
Username = "MX5CMIDPC8OQFSBOJXO6"
Password = "oga/JJJ3E1EWT+xvUF42kZqwEXrlZiDGP9FwxUca"

dataset = []
last_save_time = time.time()

class RealTimeGraph:
    def __init__(self):
        self.x_data = []  # X축 데이터 (시간)
        self.y_data = []  # Y축 데이터 (온도)
        self.data_queue = []  # 처리 대기 중인 데이터 큐
        self.max_points = 30  # 화면에 표시할 최대 데이터 포인트
        self.is_live = True  # 실시간 모드 활성화 상태
        self.current_index = 0  # 현재 보여주는 데이터의 시작 인덱스
        self.max_temp = None  # 최고 온도
        self.min_temp = None  # 최저 온도

        # 초기화
        self.fig, self.ax = plt.subplots(figsize=(10, 6))  # 그래프 크기 설정
        self.fig.subplots_adjust(right=0.8)  # 오른쪽 여백 추가
        self.line, = self.ax.plot([], [], lw=1, label='Temperature')
        self.ax.set_title("Real-Time Temperature Data")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.set_ylim(18, 27)
        plt.grid(axis='y')

        # 버튼 영역 설정
        self.buttons = {
            "left": self.fig.add_axes([0.85, 0.2, 0.04, 0.06]),  # 좌측 버튼 위치
            "right": self.fig.add_axes([0.91, 0.2, 0.04, 0.06]),  # 우측 버튼 위치
            "latest": self.fig.add_axes([0.85, 0.1, 0.1, 0.06]),  # 최신 버튼 위치
        }

        self.left_button = self.buttons["left"].text(0.5, 0.5, " ← ", ha="center", va="center", fontsize=10)
        self.right_button = self.buttons["right"].text(0.5, 0.5, " → ", ha="center", va="center", fontsize=10)
        self.latest_button = self.buttons["latest"].text(0.5, 0.5, "Go Latest", ha="center", va="center", fontsize=10)

        # 축 눈금을 제거하고 테두리는 유지
        for button_ax in self.buttons.values():
            button_ax.set_xticks([])  # x축 눈금 제거
            button_ax.set_yticks([])  # y축 눈금 제거

        # 클릭 이벤트 연결
        self.fig.canvas.mpl_connect("button_press_event", self.on_button_click)

        # 최고/최저 온도 표시 텍스트
        self.max_temp_text = self.fig.text(0.9, 0.5, "Max Temp: N/A", ha="center", va="center", fontsize=10, color="red")
        self.min_temp_text = self.fig.text(0.9, 0.4, "Min Temp: N/A", ha="center", va="center", fontsize=10, color="blue")

    def init_graph(self):
        """그래프 초기화 함수"""
        self.line.set_data([], [])
        return self.line,

    def external_update(self, time_str, temperature):
        """외부 데이터를 업데이트"""
        if temperature and isinstance(temperature, list):  # 온도가 리스트인지 확인
            self.data_queue.append((time_str, temperature))  # 데이터를 큐에 추가
            if self.is_live:
                self.update_live_graph()  # 실시간 모드일 경우 그래프 즉시 업데이트

    def update_live_graph(self):
        """실시간 모드에서 그래프 업데이트"""
        if self.data_queue:
            time_str, temperature = self.data_queue.pop(0)
            temp_value = temperature[0]  # 첫 번째 온도 값만 사용
            self.x_data.append(time_str)
            self.y_data.append(temp_value)

            # 최고/최저 온도 업데이트
            if self.max_temp is None or temp_value > self.max_temp:
                self.max_temp = temp_value
            if self.min_temp is None or temp_value < self.min_temp:
                self.min_temp = temp_value

            if self.is_live:
                self.current_index = max(0, len(self.x_data) - self.max_points)

            self.update_graph_view()

    def update_graph_view(self):
        """현재 인덱스를 기준으로 그래프 업데이트"""
        start_idx = self.current_index
        end_idx = min(len(self.x_data), start_idx + self.max_points)

        x_visible = self.x_data[start_idx:end_idx]
        y_visible = self.y_data[start_idx:end_idx]

        self.line.set_data(range(len(x_visible)), y_visible)

        # x축 레이블 업데이트
        self.ax.set_xlim(0, len(x_visible))
        self.ax.set_xticks(range(len(x_visible)))
        self.ax.set_xticklabels(x_visible, rotation=45, ha="right")

        # 최고/최저 온도 텍스트 업데이트
        self.max_temp_text.set_text(
            f"Max Temp: {self.max_temp:.1f}°C" if self.max_temp is not None else "Max Temp: N/A")
        self.min_temp_text.set_text(
            f"Min Temp: {self.min_temp:.1f}°C" if self.min_temp is not None else "Min Temp: N/A")

        self.fig.canvas.draw()

    def update_graph(self, i):
        """그래프를 업데이트 (FuncAnimation 호출)"""
        if self.is_live:
            self.update_live_graph()
        return self.line,

    def on_button_click(self, event):
        """버튼 클릭 이벤트 핸들러"""
        if event.inaxes == self.buttons["left"]:
            self.move_left()
        elif event.inaxes == self.buttons["right"]:
            self.move_right()
        elif event.inaxes == self.buttons["latest"]:
            self.go_to_latest()

    def move_left(self):
        self.is_live = False  # 실시간 모드 비활성화
        self.current_index = max(0, self.current_index - self.max_points)
        self.update_graph_view()

    def move_right(self):
        self.is_live = False  # 실시간 모드 비활성화
        self.current_index = min(len(self.x_data) - self.max_points, self.current_index + self.max_points)
        self.update_graph_view()

    def go_to_latest(self):
        """최신 데이터로 이동"""
        self.is_live = True  # 실시간 모드 활성화
        self.current_index = max(0, len(self.x_data) - self.max_points)
        self.update_graph_view()

    def start_animation(self):
        """애니메이션 시작"""
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update_graph,
            init_func=self.init_graph,
            interval=1000,
            blit=True
        )
        plt.show()

# 이하 코드 유지
# 데이터 처리 및 MQTT 관련 기존 코드 생략...



# 엑셀 데이터 저장 함수
def save_to_excel():
    global dataset
    if dataset:
        # 현재 시간에 기반한 파일 이름 생성
        current_hour = datetime.now().strftime("%Y-%m-%d_%H")
        excel_file = f"mqtt_messages_{current_hour}.xlsx"

        try:
            # 파일이 이미 존재하면 기존 데이터에 추가
            if os.path.exists(excel_file):
                existing_df = pd.read_excel(excel_file)
                new_df = pd.DataFrame(dataset)
                combined_df = pd.concat([existing_df, new_df], ignore_index=True)
                combined_df.to_excel(excel_file, index=False)
            else:
                # 새 파일 생성 및 데이터 저장
                new_df = pd.DataFrame(dataset)
                new_df.to_excel(excel_file, index=False)

            print(f"Data saved to {excel_file}")
            dataset = []  # 저장 후 데이터셋 초기화
        except Exception as e:
            print(f"Error saving to Excel: {e}")


# Base64 데이터를 정확히 변환하는 함수
def base64_to_decimal_exact(base64_string):
    try:
        # Decode Base64 to binary data
        binary_data = base64.b64decode(base64_string)

        # Convert binary data to hexadecimal string
        hex_data = binary_data.hex().upper()

        # Initialize variables
        decimal_values = []
        sum_idx = 0
        number_of_dec = 46
        split_unit = {
            10: 8, 11: 8, 14: 4, 28: 4, 29: 4, 30: 4, 24: 4, 25: 4, 31: 4, 32: 4, 33: 4, 19: 2
        }

        # Iterate and process each field
        for i in range(1, number_of_dec + 1):
            unit = split_unit.get(i, 2)  # Default to 2 bytes
            hex_value = hex_data[sum_idx:sum_idx + unit]

            if not hex_value:  # Handle empty strings gracefully
                decimal_values.append(0)
            elif i in split_unit and unit > 2:
                # Process signed bit if necessary
                decimal_values.append(process_sign_bit(hex_value))
            else:
                decimal_values.append(int(hex_value, 16))
            sum_idx += unit
        return decimal_values

    except Exception as e:
        print(f"Error in Base64 to Decimal conversion: {e}")
        return None

# 부호 비트 처리 함수
def process_sign_bit(input_hex):
    try:
        binary_data = ''.join(f"{int(digit, 16):04b}" for digit in input_hex)
        if binary_data[0] == '0':  # Positive number
            return int(input_hex, 16)
        else:  # Negative number (Two's complement)
            inverted = ''.join('1' if b == '0' else '0' for b in binary_data)
            return -(int(inverted, 2) + 1)
    except Exception as e:
        print(f"Error processing sign bit: {e}")
        return 0

# MQTT 메시지 처리 함수
def on_message(client, userdata, message):
    global dataset, last_save_time
    try:
        # 메시지 수신
        payload = message.payload.decode("utf-8")
        topic = message.topic
        qos = str(message.qos)
        print(f"Message received: {payload}\nTopic: {topic}\nQoS: {qos}")

        # JSON 파싱
        msg_data = json.loads(payload)
        if "G" not in msg_data:
            print("Invalid message format. Skipping.")
            return

        decoded_batch = []  # 현재 메시지에서 디코딩된 데이터를 임시 저장

        for index, base64_string in enumerate(msg_data["G"]):
            print(f"Processing Base64 data {index + 1}/{len(msg_data['G'])}")
            # Base64 패딩 보정
            missing_padding = len(base64_string) % 4
            if missing_padding != 0:
                base64_string += '=' * (4 - missing_padding)

            # Base64 변환 함수 호출
            result = base64_to_decimal_exact(base64_string)
            if result:
                print(f"Decoded Values: {result}")

                selected_values = {
                    "IMEI": f"{result[0]:02}{result[1]:02}{result[2]:02}",
                    "Date": f"{result[3]}년{result[4]}월{result[5]}일",
                    "Time": f"{result[6]}시{result[7]}분{result[8]}초",
                    "Longitude": f"{result[9] / 10000000:.6f} E" if result[9] >= 0 else f"{abs(result[9] / 10000000):.6f} W",
                    "Latitude": f"{result[10] / 10000000:.6f} N" if result[10] >= 0 else f"{abs(result[10] / 10000000):.6f} S",
                    "G": f"{result[14] / 10:.1f} g",
                    "Temperature": f"{result[29] / 10:.1f} ℃",
                    "Humidity": f"{result[30] / 10:.1f} %"
                }

                # 데이터 생성
                Time = f"{result[6]}:{result[7]}:{result[8]}"
                Temperature = [result[29] / 10]

                # 데이터 길이 확인
                print(f"Time 데이터: {Time}")
                print(f"Temperature 데이터: {Temperature}")

                # 클래스 함수 호출
                graph.external_update(Time, Temperature)

                # 변환된 데이터를 배치 리스트에 추가
                decoded_batch.append({
                    "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Topic": topic,
                    "QoS": qos,
                    **selected_values
                })
            else:
                print(f"Failed to decode Base64 data at index {index}")
                continue

        # 배치 처리된 데이터를 dataset에 추가
        dataset.extend(decoded_batch)
        print(f"Current dataset: {dataset}")

        # 일정 시간마다 엑셀 저장
        if time.time() - last_save_time >= 60:  # 1분마다 저장
            save_to_excel()
            last_save_time = time.time()

    except Exception as e:
        print(f"Error processing message: {e}")

# 메시지 핸들러
def message_handler(client, userdata, message):
    global dataset, last_save_time
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    msg = str(message.payload.decode("utf-8"))
    topic = message.topic
    qos = str(message.qos)

    print(f"\n\nReceived message\n{msg}\n{topic}\nQoS: {qos}")

    # 데이터 저장
    dataset.append({
        'Timestamp': current_time,
        'Message': msg,
        'Topic': topic,
        'QoS': qos
    })

    # 메시지를 추가적으로 처리 (중복된 작업을 재사용)
    try:
        msg_data = json.loads(msg)
        if "G" in msg_data:
            for base64_string in msg_data["G"]:
                missing_padding = len(base64_string) % 4
                if missing_padding != 0:
                    base64_string += '=' * (4 - missing_padding)

                try:
                    decimal_values = base64_to_decimal_exact(base64_string)

                    dataset.append({
                        "Timestamp": current_time,
                        "Topic": topic,
                        "QoS": qos,
                        "Decoded Decimal": decimal_values
                    })

                except Exception as e:
                    print(f"Error decoding Base64 string: {e}")

    except Exception as e:
        print(f"Error processing additional data: {e}")

    # 일정 시간마다 엑셀 저장
    if time.time() - last_save_time >= 60:  # 1분마다 저장
        save_to_excel()
        last_save_time = time.time()

def on_connect(client, userdata, flags, rc, properties):
    if rc == 0:
        print("Connected successfully")
    else:
        print(f"Connection failed with code {rc}")

def on_disconnect(client, userdata, flags, rc=0):
    print(str(rc))

def on_subscribe(client, userdata, mid, granted_qos, properties):
    print("subscribed: " + str(mid) + " " + str(granted_qos))

# 클라이언트 접속
client = mqtt.Client(client_id=DeviceID, callback_api_version=mqtt.CallbackAPIVersion.VERSION2)

# 핸들러 연결
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_subscribe = on_subscribe
client.on_message = on_message

# 유저 네임
client.username_pw_set(username=Username, password=Password)

# 호스트 네임
client.connect(Hostname)

# 구독 토픽 지정
client.subscribe('htns/com/things/tms/hicos2/korea/tracking/004016', 2)
client.loop_start()

# 실시간 그래프 시작
graph = RealTimeGraph()
graph.start_animation()

# 무한 반복
client.loop_forever()