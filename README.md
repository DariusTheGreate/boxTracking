# Инструкция:

## Объект внедрения:
  Учалы, К11/upper-obmotchik

## Задача:

Скрипты chuli.py и framework обнаруживают движение на 5 __участках__ с видео и в случае обнаружения - отдают управление функциям в файле functional_chuli.py, для каждого участка - своя функция. В данный момент функции логируют каждое событие в текстовый файл.

  - 1 участок - переход с пинцета.
  - 2 участок - заезд в обмотчик.
  - 3 участок - выезд из обмотчика.
  - 4 участок - попадание на виллы.

## Связь с back-ом

Работа с backend-ом осуществляется в файле functinal.py в каждой функции нужно вставить необходимый вызов в back

## Архитектура:

Участок - класс Box - координаты на видосе

Участок нужно передать одному из контроллеров

Контроллер - класс Controller и его наследники, сейчас там два типа контроллеров:
  1) MotionContourDetectorController - контролирует логирование(функция передается в конструкторе, по умолчанию прост пишет в консоль) при обнаружении движения, используя обнаружение цвета и контуры
  2) MotionFeatureDetectorController - контролирует логирование при обнаружении движения, используя фичи  

Котнроллеры используют детекторы (FeatureBasedMotionDetector, ContourColorBasedMotionDetector, AbsDiffBasedMotionDetector, etc) - сущности, которые говорят есть ли движение


