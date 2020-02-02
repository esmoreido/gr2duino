# gr2duino
Преврати свою ГР-21 в настоящий измеритель скорости потока!

![Общий вид](https://i.ibb.co/xH8x4yH/2019-10-24-18-25-16.jpg)

# Описание проекта
Гидрометрическая вертушка ГР-21 - легендарный электромеханический измеритель скорости потока в открытых руслах производства КБ Гидрометприбор (г. Ленинград - Санкт-Петербург). Ее схема очень проста и достаточно надежна, но исполнение несколько устарело. Цель настоящего проекта - создание блока регистрации оборотов лопасти вертушки на базе микроконтроллера Arduino и датчика тока ACS712. Датчик тока регистрирует силу тока в контуре подключенной к батарейке вертушки при замыкании контактов. Скетч запускает измерение, считает количество замыканий в течение 100 секунд и рассчитывает количество оборотов в секунду, которое пересчитывается в скорость течения в метрах в секунду по известным для данной вертушки коэффициентам линейной зависимости. После окончания измерения скорость выдается на дисплей. 
Проект направлен на воспроизведение любым желающим. 
## Особенности проекта
* Непосредственное измерение скорости течения в м/с
* Герметичный корпус
* Не требует поверки - официальной поверки требует только сама вертушка
* Может быть неоднократно воспроизведен любым желающим (при наличии некоторых навыков работы с Arduino и паяльником)
* Низкая* стоимость сборки

_*Относительно рыночной стоимости аналогичного прибора, при условии самостоятельного изготовления и без учета стоимости самой вертушки ГР-21 (которой нет в продаже, зато наверняка очень много на складах различных учреждений)._
## Важное замечание
Проект является личной инициативой автора. Использованные детали (включая саму вертушку ГР-21) являются личной собственностью автора. Проект будет дорабатываться. Любые предложения, замечания и прочие полезные комментарии очень приветствуются (здесь или на почту [vsevolod.moreydo@iwp.ru](mailto:vsevolod.moreydo@iwp.ru)).

# Принципиальная схема
![Схема компонентов](https://i.ibb.co/kKr3jx5/gr2duino-bb.png)

# Как это работает
1. Собранный блок регистратора с установленными батарейками подключить проводом к разъемам вертушки. 
2. Включить блок регистратора отдельной кнопкой включения.
3. Нажать и удерживать кнопку начала измерения до первого замыкания контактов, после чего включится таймер и счетчик оборотов.
4. По истечении 100 секунд регистратор выведет на дисплей значение измеренной скорости течения в м/с.
Для повторного измерения - повторить п. 3-4.

# Сделай сам
Для сборки вам понадобятся нижеперечисленные компоненты и скачанный скетч из этого репозитория. 
Для первой сборки были использованы компоненты производства компании "Амперка". Это сделано, преимущественно, для удобства сборки - модульные компоненты Troyka очень просты в соединении. При сборке первого прототипа пайка использовась только для соединения кнопок. Стоимость проекта можно снизить примерно вдвое при использовании аналогичных компонентов, заказанных на aliexpress. 
Список деталей для проекта (цены указаны на конец 2019 года):
* [Iskra Neo  - 990 р. ](https://amperka.ru/product/iskra-neo)
* [Датчик тока ACS712  - 440 р. ](https://amperka.ru/product/troyka-current-sensor)
* [Troyka Slot Shield v2  - 590 р. ](https://amperka.ru/product/arduino-troyka-slot-shield)
* [Текстовый дисплей 16×2 (Troyka-модуль)  - 890 р. ](https://amperka.ru/product/troyka-display-lcd-text-16x2)
* [Герметичный корпус 120×120×60  - 790 р. ](https://amperka.ru/product/sealed-enclosure-120x120x60)
* [Крепления Arduino и Iskra (#Структор) - 290 р. ](https://amperka.ru/product/structor-arduino)
* [Корпусная кнопка  - 120 р. ](https://amperka.ru/product/button_for_boxes_17mm)
* [Рокерный выключатель с подсветкой  - 70 р. ](https://amperka.ru/product/switch_button_with_light)
* [Кабель питания от батарейки Крона 2 шт - 70 р. ](https://amperka.ru/product/krona-21mm-cable)
* [Батарейка Крона 9V 2 шт - 500 р. ]()
* [Аудиоразъем с 2 подпружиненными контактами  - 200 р. ](https://yandex.ru/images/search?text=%D0%B0%D1%83%D0%B4%D0%B8%D0%BE%20%D1%80%D0%B0%D0%B7%D1%8A%D0%B5%D0%BC%20WP&from=tabbar)
Итого на конец 2019 года - 4950 р. 
## Выявленные особенности и недостатки (на начало 2020 г.)
* Arduino Leonardo, возможно, несколько избыточна для данного проекта, можно использовать более миниатюрный и дешевый контроллер
* Схема питания с 2 батарейками не является идеальной. ~~Тем не менее, авторские попытки исправить это пока не привели к успеху - при питании контура вертушки от разъемов 5V Aduino замыкание контактов вертушки приводит к короткому замыканию и перезагрузке контроллера. Автор будет признателен за совет по устранению этого минуса.~~ Автор уже нашел решение, но пока его не воплотил. Ждите обновлений.
* Регистратор чувствителен к "свежести" батареек. За пороговое значение силы тока, при котором регистрируется оборот лопасти, отвечает константа ```thresholdCurrent```, которая по умолчанию равно 0.5 мА. По опыту автора, новая батарейка Крона выдает более 1 мА тока, что является достаточным, а уже бывшая в употреблении - не менее 0.75 мА.
* Использованный аудиоразъем для подключения провода вертушки понравился удобством присоединения проводов, но его долговечность толком не испытана
