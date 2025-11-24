#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <random>
#include <cmath>

namespace gazebo
{
    class RandomMovementPlugin : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        
        // Параметры поля и объекта
        double field_size_x, field_size_y;
        double object_size_x, object_size_y;
        
        // Параметры движения
        double current_speed;
        double min_speed, max_speed;
        
        // Траектория
        double target_direction;
        double current_direction;
        double turn_radius;
        double max_turn_radius;
        
        // Время
        common::Time last_update_time;
        double behavior_change_time;
        double time_since_last_change;
        
        // Генератор случайных чисел
        std::default_random_engine generator;
        
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Сохраняем указатель на модель
            this->model = _parent;
            
            // Параметры из SDF
            if (_sdf->HasElement("field_size_x"))
                field_size_x = _sdf->Get<double>("field_size_x");
            else
                field_size_x = 10.0;
                
            if (_sdf->HasElement("field_size_y"))
                field_size_y = _sdf->Get<double>("field_size_y");
            else
                field_size_y = 10.0;
                
            if (_sdf->HasElement("object_size_x"))
                object_size_x = _sdf->Get<double>("object_size_x");
            else
                object_size_x = 1.0;
                
            if (_sdf->HasElement("object_size_y"))
                object_size_y = _sdf->Get<double>("object_size_y");
            else
                object_size_y = 1.0;
            
            // Инициализация параметров движения
            min_speed = 0.5;
            max_speed = 1.5;
            current_speed = (min_speed + max_speed) / 2.0;
            
            max_turn_radius = 3.0;
            turn_radius = max_turn_radius;
            
            // Начальное направление
            std::uniform_real_distribution<double> dir_dist(0, 2 * M_PI);
            current_direction = dir_dist(generator);
            target_direction = current_direction;
            
            // Время
            last_update_time = common::Time::GetWallTime();
            behavior_change_time = 3.0;
            time_since_last_change = 0.0;
            
            // Соединяем с циклом обновления
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RandomMovementPlugin::OnUpdate, this));
                
            gzmsg << "RandomMovementPlugin loaded for model: " << model->GetName() << std::endl;
        }
        
        void OnUpdate()
        {
            common::Time current_time = common::Time::GetWallTime();
            double dt = (current_time - last_update_time).Double();
            
            if (dt <= 0) return;
            
            last_update_time = current_time;
            time_since_last_change += dt;
            
            // Обновляем поведение
            UpdateBehavior(dt);
            
            // Применяем движение
            ApplyMovement(dt);
        }
        
    private:
        void UpdateBehavior(double dt)
        {
            // Меняем скорость случайным образом
            if (time_since_last_change > behavior_change_time)
            {
                std::uniform_real_distribution<double> speed_dist(min_speed, max_speed);
                std::uniform_real_distribution<double> dir_change_dist(-M_PI/4, M_PI/4);
                
                current_speed = speed_dist(generator);
                
                // Выбираем новое направление
                double new_direction = current_direction + dir_change_dist(generator);
                
                // Нормализуем угол
                while (new_direction > 2 * M_PI) new_direction -= 2 * M_PI;
                while (new_direction < 0) new_direction += 2 * M_PI;
                
                target_direction = new_direction;
                
                // Рассчитываем безопасный радиус поворота
                CalculateSafeTurnRadius();
                
                // Сбрасываем таймер
                time_since_last_change = 0.0;
                behavior_change_time = 2.0 + (std::uniform_real_distribution<double>(0, 3)(generator));
            }
            
            // Плавно изменяем направление
            double angle_diff = target_direction - current_direction;
            
            // Находим кратчайший путь поворота
            if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            if (angle_diff < -M_PI) angle_diff += 2 * M_PI;
            
            // Угловая скорость на основе радиуса поворота
            double max_angular_change = (current_speed / turn_radius) * dt;
            
            if (std::abs(angle_diff) > max_angular_change)
            {
                current_direction += (angle_diff > 0) ? max_angular_change : -max_angular_change;
            }
            else
            {
                current_direction = target_direction;
            }
            
            // Нормализуем угол
            while (current_direction > 2 * M_PI) current_direction -= 2 * M_PI;
            while (current_direction < 0) current_direction += 2 * M_PI;
        }
        
        void CalculateSafeTurnRadius()
        {
            ignition::math::Pose3d current_pose = model->WorldPose();
            double x = current_pose.Pos().X();
            double y = current_pose.Pos().Y();
            
            // Минимальный радиус из соотношения скорость/радиус
            double min_radius_from_speed = current_speed / (M_PI/2); // Макс угловая скорость ~90°/с
            
            // Рассчитываем безопасные расстояния до границ
            double safe_distance_left = x - object_size_x/2;
            double safe_distance_right = field_size_x - (x + object_size_x/2);
            double safe_distance_bottom = y - object_size_y/2;
            double safe_distance_top = field_size_y - (y + object_size_y/2);
            
            // Безопасный радиус на основе расстояния до границ
            double safe_radius_from_bounds = std::min({
                safe_distance_left, 
                safe_distance_right,
                safe_distance_bottom,
                safe_distance_top
            });
            
            // Выбираем радиус поворота
            turn_radius = std::max(min_radius_from_speed, 
                                 std::min(max_turn_radius, safe_radius_from_bounds));
            
            // Обеспечиваем минимальный радиус
            turn_radius = std::max(turn_radius, 0.3);
        }
        
        void ApplyMovement(double dt)
        {
            ignition::math::Pose3d current_pose = model->WorldPose();
            double x = current_pose.Pos().X();
            double y = current_pose.Pos().Y();
            
            // Рассчитываем новую позицию
            double new_x = x + current_speed * std::cos(current_direction) * dt;
            double new_y = y + current_speed * std::sin(current_direction) * dt;
            
            bool collision_x = false;
            bool collision_y = false;
            
            // Проверяем границы с учетом bounding box
            if (new_x - object_size_x/2 < 0)
            {
                collision_x = true;
                new_x = object_size_x/2;
            }
            else if (new_x + object_size_x/2 > field_size_x)
            {
                collision_x = true;
                new_x = field_size_x - object_size_x/2;
            }
            
            if (new_y - object_size_y/2 < 0)
            {
                collision_y = true;
                new_y = object_size_y/2;
            }
            else if (new_y + object_size_y/2 > field_size_y)
            {
                collision_y = true;
                new_y = field_size_y - object_size_y/2;
            }
            
            // Если было столкновение с границей, выбираем новое направление
            if (collision_x || collision_y)
            {
                GenerateNewDirectionAfterCollision(collision_x, collision_y, x, y);
                CalculateSafeTurnRadius();
                
                // Немедленно начинаем поворот
                time_since_last_change = 0.0;
                behavior_change_time = 1.0; // Быстрее сменим поведение после столкновения
            }
            
            // Устанавливаем новую позицию и ориентацию
            ignition::math::Pose3d new_pose(
                ignition::math::Vector3d(new_x, new_y, current_pose.Pos().Z()),
                ignition::math::Quaterniond(0, 0, current_direction)
            );
            
            model->SetWorldPose(new_pose);
        }
        
        void GenerateNewDirectionAfterCollision(bool collision_x, bool collision_y, double current_x, double current_y)
        {
            std::uniform_real_distribution<double> small_angle_dist(-M_PI/3, M_PI/3);
            std::uniform_real_distribution<double> large_angle_dist(M_PI/4, 3*M_PI/4);
            
            // Определяем, в каком углу мы находимся
            bool in_corner = (collision_x && collision_y);
            
            if (in_corner)
            {
                // В углу - выбираем направление, ведущее к центру поля
                double center_x = field_size_x / 2.0;
                double center_y = field_size_y / 2.0;
                double direction_to_center = std::atan2(center_y - current_y, center_x - current_x);
                
                // Добавляем случайное отклонение
                target_direction = direction_to_center + small_angle_dist(generator);
            }
            else if (collision_x)
            {
                // Столкновение с вертикальной границей
                // Выбираем направление, параллельное границе с небольшим отклонением внутрь
                if (current_x < field_size_x / 2) // Левая граница
                    target_direction = small_angle_dist(generator); // Вправо с отклонением
                else // Правая граница
                    target_direction = M_PI + small_angle_dist(generator); // Влево с отклонением
            }
            else if (collision_y)
            {
                // Столкновение с горизонтальной границей
                // Выбираем направление, параллельное границе с небольшим отклонением внутрь
                if (current_y < field_size_y / 2) // Нижняя граница
                    target_direction = M_PI/2 + small_angle_dist(generator); // Вверх с отклонением
                else // Верхняя граница
                    target_direction = -M_PI/2 + small_angle_dist(generator); // Вниз с отклонением
            }
            
            // Нормализуем угол
            while (target_direction > 2 * M_PI) target_direction -= 2 * M_PI;
            while (target_direction < 0) target_direction += 2 * M_PI;
            
            // Убеждаемся, что новое направление не ведет сразу обратно в границу
            EnsureSafeDirection(current_x, current_y);
        }
        
        void EnsureSafeDirection(double current_x, double current_y)
        {
            // Проверяем, не ведет ли новое направление сразу обратно в границу
            double test_distance = 1.0;
            double test_x = current_x + test_distance * std::cos(target_direction);
            double test_y = current_y + test_distance * std::sin(target_direction);
            
            bool would_collide_x = (test_x - object_size_x/2 < 0) || (test_x + object_size_x/2 > field_size_x);
            bool would_collide_y = (test_y - object_size_y/2 < 0) || (test_y + object_size_y/2 > field_size_y);
            
            if (would_collide_x || would_collide_y)
            {
                // Корректируем направление
                std::uniform_real_distribution<double> correction_dist(M_PI/6, M_PI/3);
                target_direction += correction_dist(generator);
                
                // Нормализуем угол
                while (target_direction > 2 * M_PI) target_direction -= 2 * M_PI;
                while (target_direction < 0) target_direction += 2 * M_PI;
            }
        }
    };
    
    GZ_REGISTER_MODEL_PLUGIN(RandomMovementPlugin)
}