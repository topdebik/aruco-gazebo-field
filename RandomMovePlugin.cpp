#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <random>
#include <chrono>

namespace gazebo
{
    class RandomMovePlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Сохраняем указатель на модель
            this->model = _parent;
            
            // Получаем размеры поля из параметров SDF
            if (_sdf->HasElement("field_width"))
                field_width = _sdf->Get<double>("field_width");
            if (_sdf->HasElement("field_height"))
                field_height = _sdf->Get<double>("field_height");
            
            // Получаем размеры объекта из параметров SDF
            if (_sdf->HasElement("object_width"))
                object_width = _sdf->Get<double>("object_width");
            if (_sdf->HasElement("object_height"))
                object_height = _sdf->Get<double>("object_height");
            
            // Получаем начальную позицию объекта и сохраняем высоту
            ignition::math::Pose3d initial_pose = this->model->WorldPose();
            current_x = initial_pose.Pos().X();
            current_y = initial_pose.Pos().Y();
            fixed_height = initial_pose.Pos().Z(); // Сохраняем исходную высоту
            
            // Инициализируем генератор случайных чисел
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            generator.seed(seed);
            
            // Устанавливаем начальное направление
            SetRandomDirection();
            
            // Устанавливаем начальную скорость
            UpdateSpeed();
            
            // Подключаемся к циклу обновления мира
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RandomMovePlugin::OnUpdate, this));
            
            gzmsg << "RandomMovePlugin loaded for model: " << this->model->GetName() << std::endl;
            gzmsg << "Field size: " << field_width << " x " << field_height << " (from 0,0 to " << field_width << "," << field_height << ")" << std::endl;
            gzmsg << "Object size: " << object_width << " x " << object_height << std::endl;
            gzmsg << "Initial position: " << current_x << ", " << current_y << ", height: " << fixed_height << std::endl;
        }

    private:
        void OnUpdate()
        {
            // Получаем время с последнего обновления
            common::Time current_time = this->model->GetWorld()->SimTime();
            double delta_time = (current_time - last_update_time).Double();
            
            if (delta_time <= 0.0)
            {
                last_update_time = current_time;
                return;
            }
            
            // Обновляем скорость каждые 2-5 секунд
            if ((current_time - last_speed_update_time).Double() > speed_update_interval)
            {
                UpdateSpeed();
                last_speed_update_time = current_time;
                
                // Случайно изменяем интервал обновления скорости (2-5 секунд)
                std::uniform_int_distribution<int> interval_dist(2, 5);
                speed_update_interval = interval_dist(generator);
            }
            
            // Обновляем направление каждые 3-8 секунд или при приближении к границе
            if ((current_time - last_direction_update_time).Double() > direction_update_interval ||
                IsNearBoundary())
            {
                SetRandomDirection();
                last_direction_update_time = current_time;
                
                // Случайно изменяем интервал обновления направления (3-8 секунд)
                std::uniform_int_distribution<int> interval_dist(3, 8);
                direction_update_interval = interval_dist(generator);
            }
            
            // Применяем движение
            MoveObject(delta_time);
            
            last_update_time = current_time;
        }
        
        void MoveObject(double delta_time)
        {
            // Рассчитываем новую позицию
            double new_x = current_x + current_velocity.X() * delta_time;
            double new_y = current_y + current_velocity.Y() * delta_time;
            
            // Проверяем границы с учетом bounding box объекта
            bool collision_x = false;
            bool collision_y = false;
            
            // Проверка границ по X (левая и правая границы)
            if (new_x - object_width/2 < 0)
            {
                new_x = object_width/2;
                collision_x = true;
            }
            else if (new_x + object_width/2 > field_width)
            {
                new_x = field_width - object_width/2;
                collision_x = true;
            }
            
            // Проверка границ по Y (нижняя и верхняя границы)
            if (new_y - object_height/2 < 0)
            {
                new_y = object_height/2;
                collision_y = true;
            }
            else if (new_y + object_height/2 > field_height)
            {
                new_y = field_height - object_height/2;
                collision_y = true;
            }
            
            // Обрабатываем отскок от границ
            if (collision_x)
            {
                current_velocity.X(-current_velocity.X());
                // Немного изменяем направление Y при отскоке для более естественного движения
                std::uniform_real_distribution<double> adjust_dist(-0.5, 0.5);
                current_velocity.Y(current_velocity.Y() + adjust_dist(generator));
            }
            
            if (collision_y)
            {
                current_velocity.Y(-current_velocity.Y());
                // Немного изменяем направление X при отскоке
                std::uniform_real_distribution<double> adjust_dist(-0.5, 0.5);
                current_velocity.X(current_velocity.X() + adjust_dist(generator));
            }
            
            // Нормализуем скорость после возможных изменений
            double speed = current_velocity.Length();
            if (speed > 0)
            {
                current_velocity = current_velocity.Normalize() * current_speed;
            }
            
            // Обновляем текущие координаты
            current_x = new_x;
            current_y = new_y;
            
            // Вычисляем угол поворота объекта по направлению движения
            double yaw = atan2(current_velocity.Y(), current_velocity.X()) + M_PI_2;
            
            // Создаем новую позу с фиксированной высотой
            ignition::math::Pose3d new_pose(current_x, current_y, fixed_height, 0, 0, yaw);
            
            // Устанавливаем новую позицию и ориентацию
            this->model->SetWorldPose(new_pose);
        }
        
        void SetRandomDirection()
        {
            std::uniform_real_distribution<double> angle_dist(0, 2 * M_PI);
            double angle = angle_dist(generator);
            
            direction.X(cos(angle));
            direction.Y(sin(angle));
            direction.Normalize();
            
            // Обновляем текущую скорость с новым направлением
            current_velocity = direction * current_speed;
            
            gzmsg << "New direction: " << direction.X() << ", " << direction.Y() 
                  << " Speed: " << current_speed << " m/s" << std::endl;
        }
        
        void UpdateSpeed()
        {
            std::uniform_real_distribution<double> speed_dist(0.5, 1.5);
            current_speed = speed_dist(generator);
            
            // Обновляем текущую скорость
            current_velocity = direction * current_speed;
        }
        
        bool IsNearBoundary()
        {
            // Проверяем приближение к границам (в пределах 1 метра)
            double boundary_margin = 1.0;
            
            return (current_x - object_width/2 <= boundary_margin) ||
                   (current_x + object_width/2 >= field_width - boundary_margin) ||
                   (current_y - object_height/2 <= boundary_margin) ||
                   (current_y + object_height/2 >= field_height - boundary_margin);
        }

    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        
        // Параметры поля и объекта
        double field_width = 10.0;    // метры (от 0 до field_width)
        double field_height = 10.0;   // метры (от 0 до field_height)
        double object_width = 0.5;    // метры
        double object_height = 0.5;   // метры
        
        // Текущие координаты объекта
        double current_x = 0.0;
        double current_y = 0.0;
        double fixed_height = 0.5;    // Фиксированная высота объекта
        
        // Переменные движения
        ignition::math::Vector3d direction;
        ignition::math::Vector3d current_velocity;
        double current_speed = 1.0;
        
        // Генератор случайных чисел
        std::default_random_engine generator;
        
        // Временные метки для обновления
        common::Time last_update_time;
        common::Time last_speed_update_time;
        common::Time last_direction_update_time;
        double speed_update_interval = 3.0;    // секунды
        double direction_update_interval = 5.0; // секунды
    };

    // Регистрируем плагин
    GZ_REGISTER_MODEL_PLUGIN(RandomMovePlugin)
}