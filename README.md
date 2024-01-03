# Pitch-PID-controller
## Задача: реализовать систему стабилизации углового положения ЛА мультироторного типа на основе 2 каскадных ПИД-регуляторов, коэффициенты которых необходимо настроить.
## Программа состоит из длинамической модели, описывающей ЛА, двух каскадных ПИД-регуляторов (для углового положения и угловой скорости), заставляющих аппарат отклоняться на угол(в данной задаче 30 градусов = 0.52 радиан), симулятора, запускающего процесс стабилизации на определенный отрезок времени. 
### Графики изменений углового положения, угловой скорости, углового ускорения. Из графиков видно, что переходный процесс установления угла = 0.52 радиан занимает меньше 1 секунды. 
<img src="images/pitch-PID.png" width="1000" height="500"/>

# UAV-simulator
## Задача: Запрограммировать математическую модель БЛА в виде функции правых частей в форме Коши (для дальнейшего численного решения системы дифференциальных уравнений модели мультикоптера), для выбранной математической модели реализована система управления с использованием 12 каскадных ПИД-регуляторов в качестве системы регулирования, разработан класс симулятора, который отвечает за вызов методов для расчёта управляющих воздействий, правых частей нашей математической модели и отправку сообщений для визуализации результата.
### MathModel.py содержит математическую реализацию дрона
### constants.py содержит все физические данные модели дрона
### controller.py содержит класс ПИД-регулятора и системы управления для всего БЛА ввиде 12 каскадных пидов
### simulator.py содержит класс симулятора БЛА, который осуществляет подключение к UDP порту, для передачи вектора состояния БЛА, осуществляет вычисление управляющего воздействия, по этому управляющему воздействию осуществляет пересчет значений математической модели
### main.py - участок программы, в котором создаются классы контроллера, математической модели, симулятора, которые необходимы для симуляции работы БЛА. Также выставляются коэффициенты 12 ПИД-регуляторов, задается миссия в виде набора точек.
## Тестирование работы программы происходило на симуляторе в папке scripts/vizualizer.py.
### График изменения состояния БЛА по всем координатам в ходе полета по точкам [0,0,10,3],[0,0,10,1],[5,1,7,1],[6,6,6,0.5],[6,5,10,2],[3,0,10,-1],[1,1,1,0],[10,10,1,0],[5,2,5,1]
<img src="images/uav-simulator.png" width="1000" height="500"/>

### Видео работы: https://drive.google.com/file/d/1uK8oWkgHODc29t3avhAko8V5KXbvPtUL/view?usp=sharing


