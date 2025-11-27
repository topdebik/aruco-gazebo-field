## aruco-gazebo-field
# Порядок запуска
1. Изменить все абсолютные пути в файле my_clover_description/launch/aruco_world_with_drone.launch
2. Инициализировать переменные окружения (требуется при каждом создании нового терминала, где будет запускаться симуляция):
```bash
source setup_world.sh
```
3. Запустить симуляцию
```bash
clover my_clover_description/launch/aruco_world_with_drone.launch
```

# Запуск алгоритма для дрона
1. Войти в папку clover4_car_tracker
2. Установить зависимости Python:
```bash
pip install --upgrade -r requirements.txt
```
3. Запустить алгоритм:
```bash
python3 main.py
```