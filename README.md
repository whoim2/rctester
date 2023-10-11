# rctester
arduino-based RC tester (servo-pwm, ppm, sbus signals)


Помощник в отладке RC-моделей. Удобнее, чем продающиеся существующие.
Два режима работы: генерация PWM(Servo)/SBUS или PPM (на выводе PWM/Servo). Построен на базе arduino nano / mini с mcu atmega168 / atmega328.

Вывод осуществляется на OLED 128*32 дисплей ssd1306, индицируется RC-значение 544..2200 и напряжение питания сервы/приемника.

Органы управления: потенциометр значения RC с программной фильтрацией и кнопка. Короткое нажатие на кнопку устанавливает центр значения RC (1500), длинное при минимальном значении RC переключает режим (PWM/SBUS или PPM), при прочих значениях RC
изменяет канал вывода RC (1..4), для удобства отладки готовой RC-модели.

Напряжение питания устанавливается отдельным потенциометром и настраиваемым BEC (DC-DC преобразователем с подстроечником, где подстроечник выпаян и заменен на внешний потенциометр). 

Калибровка вольтметра: устанавливаем (расскоментируем) дефайн VM_CALIBRATE, включаем прибор. Вместо напряжения показывается усредненное значение ADC. Вписываем его в дефайн VM_MEASURED_N. В соседний VM_MEASURED_V вписываем измеренное вольтметром напряжение питания.
Напряжение питания заводится на пин A6 через резисторный делитель, рассчитанный на понижение напряжения от максимально планируемого (обычно 2S) до рабочего напряжения питания arduino (5v или 3.3v в зависимости от версии). Не забываем, что сама arduino питается от того же
напряжения через встроенный LDO-стабилизатор, имеющий ограничение по верхней планке напряжения.

Кнопка подключается между A1 и GND. Потенциометр выбора значения RC заводится на пин A0, крайние пины - между GND и 5v/3.3v платы (рабочее напряжение питания arduino). 

Выводы pwm/ppm и sbus в случае использования 5v arduino рекомендуется выводить также через резисторный делитель, чтобы понизить до 3.3в, так как подавляющее число современных полетников работают именно с этим логическим уровнем. Калькуляторов резисторов в интернете полно.

Схему и фотки немного попозже.
