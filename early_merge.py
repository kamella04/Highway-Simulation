# Early merge
import random as rand
import numpy as np
import random
from matplotlib.animation import FuncAnimation
from threading import Thread
import time
from matplotlib import pyplot as plt
import math
import os

#Constants
MEAN_SPEED_HIGHWAY = 27.78  # Mean speed on the highway
STDEV_SPEED = 3.5  # Standard deviation of car speeds
MEAN_SPEED_RAMP = 45/3.6  # Mean speed on the ramp
NUMBER_OF_CARS = 10  # Number of cars in the simulation
DECELERATION_RATE = 6/3.6  # Deceleration rate when maintaining safe distance
ACCELERATION_RATE = 6/3.6
LENGTH_OF_RAMP = 100  # Length of the ramp
END_ACCELERATION_LANE = 500 # End position of acceleration lane
INTERARRIVAL_HIGHWAY = 2.5
INTERARRIVAL_RAMP = 3.5
SIM_OVER = False
ramp_cnt = 0

class Car:
  def __init__(self, speed, destination):
    global ramp_cnt
    self.speed = speed
    self.destination = destination
    self.merge = False
    self.change_lane = False
    self.car_ahead = False
    self.car_behind = False
    self.on_ramp = False
    self.on_acceleration_lane = False
    self.merged_ramp = False
    self.destination_A=False
    self.destination_B=False
    self.changed_destination=False
    self.need_to_change_destination=False
    self.waiting = False
    self.position = 0
    self.y=0
    self.car_length= 5.2
    self.wait_time = 0
    self.ramp_queue_no = ramp_cnt
    self.waiting_sharp_merge = False

    ramp_cnt += 1

  def accelerate(self):
    if self.speed < 15:
      self.speed += random.expovariate(1/(2*ACCELERATION_RATE))
      self.speed = min(self.speed, MEAN_SPEED_HIGHWAY)
    else:
      self.speed += random.expovariate(1/ACCELERATION_RATE)
      self.speed = min(self.speed, MEAN_SPEED_HIGHWAY)


  def keeping_safe_distance(self, simulation, time_interval):

      nearest_car_ahead = None
      min_distance = float('inf')

      for car_ahead in simulation.highway:
        if not car_ahead.merge:
          if self.y - 1 <= car_ahead.y <= self.y + 4 or self.y - 1 <= car_ahead.y+3 <= self.y+4:
            if car_ahead.position > self.position and (car_ahead.position - self.position < min_distance):
                min_distance = car_ahead.position - self.position
                nearest_car_ahead = car_ahead
        else:
          if self.y - 10 <= car_ahead.y <= self.y + 4 or self.y - 10 <= car_ahead.y+3 <= self.y+4:
            if car_ahead.position > self.position and (car_ahead.position - self.position < min_distance):
                min_distance = car_ahead.position - self.position
                nearest_car_ahead = car_ahead

      if nearest_car_ahead:
          safe_distance = self.speed * 2 + self.car_length  # Applying the 2-second rule
          current_distance = min_distance - self.car_length

          if current_distance < safe_distance:
              if current_distance < 1 + self.car_length:
                self.speed = 0
              # Decelerate to maintain safe distance
              return False
          elif current_distance > safe_distance:
              return True


      return True


  def move(self, simulation, time_interval):
    if self.on_ramp and self.position > 470 and not self.merge:
        return

    if self.position < 790:
      simulation.changed_destination(self)
      self.position += self.speed * time_interval
      if simulation.changed_destination and self.destination_A and self.y <= 110:
          #self.accelerate(time_interval)
          self.position += self.speed * time_interval
          self.y += self.speed * time_interval/1.5
          self.y = min(self.y, 110)
      elif self.changed_destination and self.destination_B and self.y >= 85:
          #self.accelerate(time_interval)
          self.position += self.speed * time_interval
          self.y -= self.speed * time_interval/1.5
          self.y = max(self.y, 85)
    else:
        self.position += self.speed * time_interval
        if self.destination_A:
            self.y += self.speed * time_interval/ 8   # divided by 8 is for correct angle
        if self.destination_B:
            self.y -= self.speed * time_interval/ 8     # divided by 8 is for angle


  def move_ramp(self,simulation, time_interval):
    if self.position < 250:
      self.on_acceleration_lane = True
      self.y+=self.speed * time_interval/3


class Simulation:
  def __init__(self):
    self.close = False
    self.highway = []
    self.upstream = []
    self.lane_A = []
    self.lane_B = []
    self.ramp = []
    self.merged_ramp = []
    self.t = 0
    self.timesteps = []
    self.interval = 0.1
    self.waiting_list = []
    self.lane_A_length = []
    self.lane_B_length = []
    self.ramp_length= []
    self.lane_A_speed = 0
    self.lane_B_speed = 0
    self.lane_A_speed_list=[]
    self.lane_B_speed_list=[]

    self.fig = plt.figure()
    self.fig.set_size_inches(10, 2)
    self.axis = plt.axes(xlim=(0, 1000), ylim=(0, 200))
    self.anim = None

    self.line, = self.axis.plot([], [], lw=2)
    self.axis.plot([0, 250], [75, 75], color='black', linestyle='solid')
    self.axis.plot([250, 500], [75, 75], color='black', linestyle='dashed')
    self.axis.plot([500, 800], [75, 75], color='black', linestyle='solid')
    self.axis.plot([800, 1000], [75, 50], color='black', linestyle='solid')
    self.axis.plot([0, 800], [125, 125], color='black', linestyle='solid')
    self.axis.plot([800, 1000], [125, 150], color='black', linestyle='solid')
    self.axis.plot([0, 800], [100, 100], color='black', linestyle='dashed')
    self.axis.plot([800, 1000], [100, 125], color='black', linestyle='solid')
    self.axis.plot([800, 1000], [100, 75], color='black', linestyle='solid')
    self.axis.plot([25, 250], [0, 75], color='black', linestyle='solid')
    self.axis.plot([75, 250], [0, 50], color='black', linestyle='solid')
    self.axis.plot([250, 470], [50, 50], color='black', linestyle='solid')
    self.axis.plot([470, 500], [50, 75], color='black', linestyle='solid')

  def car_arrival_lane_A(self):
    global SIM_OVER
    speed = rand.uniform(max(25, MEAN_SPEED_HIGHWAY - STDEV_SPEED), min(33, MEAN_SPEED_HIGHWAY + STDEV_SPEED))

    for car in self.lane_A:
      if car.position < speed * 2:
        time.sleep(random.expovariate(1/INTERARRIVAL_HIGHWAY))
        self.car_arrival_lane_A()
        return

    car = Car(speed, destination="A")
    car.position = 0
    car.y = 110
    car.destination_A = True
    self.lane_A.append(car)
    self.upstream.append(car)
    self.highway.append(car)
    time.sleep(random.expovariate(1/INTERARRIVAL_HIGHWAY))

    if random.random() <= 0.1:  # Arbitrary probability to change destination
      car.need_to_change_destination = True
      print("Car will need to change destination")
    if not SIM_OVER:
        self.car_arrival_lane_A()

  def car_arrival_lane_B(self):
    global SIM_OVER
    speed = rand.uniform(max(25, MEAN_SPEED_HIGHWAY - STDEV_SPEED), min(33, MEAN_SPEED_HIGHWAY + STDEV_SPEED))

    for car in self.lane_B:
      if car.position < speed * 2:
        time.sleep(random.expovariate(1/INTERARRIVAL_HIGHWAY))
        self.car_arrival_lane_B()
        return

    car = Car(speed, destination="B")
    car.position = 0
    car.y = 85
    car.destination_B = True
    self.lane_B.append(car)
    self.upstream.append(car)
    self.highway.append(car)
    time.sleep(random.expovariate(1/INTERARRIVAL_HIGHWAY))


    if  random.random() <= 0.1:  # Arbitrary probability to change destination
      car.need_to_change_destination = True
      print("Car willl need to change destination")

    if not SIM_OVER:
        self.car_arrival_lane_B()

  def car_arrival_ramp(self):
    speed = rand.uniform(max(25, MEAN_SPEED_RAMP - STDEV_SPEED), min(33, MEAN_SPEED_RAMP + STDEV_SPEED))

    for car in self.ramp:
      if car.position < speed * 2:
        time.sleep(random.expovariate(1/INTERARRIVAL_RAMP))

        self.car_arrival_ramp()
        return

    car = Car(speed, destination="B")
    car.destination_B = True
    car.position = 50
    car.on_ramp = True
    self.ramp.append(car)
    self.highway.append(car)
    print(f"Ramp car generated at position {car.position} with speed {car.speed}")
    next_arrival = random.expovariate(1 / INTERARRIVAL_RAMP)

    if  random.random() <= 0.25:  # Arbitrary probability to change destination
      car.need_to_change_destination = True
      print("Car willl need to change destination")

    time.sleep(next_arrival + 0.5)
    if not SIM_OVER:
        self.car_arrival_ramp()

  def is_safe_to_change_lane(self, car):
      if not car.on_ramp:
        if car in self.lane_A:
            for car_ahead in self.lane_B:
                if car_ahead.position > car.position:
                    safe_distance = car.speed * 2
                    current_distance = car_ahead.position - car.position - 12
                    if safe_distance < current_distance:
                        return True
                    else:
                        return False
        elif car in self.lane_B:
            for car_ahead in self.lane_A:
                if car_ahead.position > car.position:
                    safe_distance = car.speed * 2
                    current_distance = car_ahead.position - car.position - 12
                    if safe_distance < current_distance:
                        return True
                    else:
                        return False
  def check_behind(self, car):
    if not car.on_ramp:
      if car in self.lane_A:
          for car_behind in self.lane_B:
              if car_behind.position < car.position:
                  safe_distance = car_behind.speed * 2
                  current_distance = car.position - car_behind.position - 12
                  if safe_distance < current_distance:
                      return True
                  else:
                      return False
      elif car in self.lane_B:
          for car_behind in self.lane_A:
              if car_behind.position < car.position:
                  safe_distance = car_behind.speed * 2
                  current_distance = car.position - car_behind.position - 12
                  if safe_distance < current_distance:
                      return True
                  else:
                      return False

  def changed_destination(self, car):
      lane_A_speed = 0
      lane_B_speed = 0
      lane_B_cnt = 0
      lane_A_cnt = 0

      cars_lane_slower = False

      for lane_car in self.lane_A:
        if lane_car.position < 790:
          lane_A_speed += lane_car.speed
          lane_A_cnt += 1
      if lane_A_cnt > 0:
        lane_A_speed /= lane_A_cnt
        self.lane_A_speed = lane_A_speed

      for lane_car in self.lane_B:
        if lane_car.position < 790:
          lane_B_speed += lane_car.speed
          lane_B_cnt += 1
      if lane_B_cnt > 0:
        lane_B_speed /= lane_B_cnt
        self.lane_B_speed = lane_B_speed

      if car.destination_A:
        if lane_B_speed - lane_A_speed > 5:
          cars_lane_slower = True
      elif car.destination_B:
        if lane_A_speed - lane_B_speed > 5:
          cars_lane_slower = True

      if cars_lane_slower and car.position < 650:
        if random.random() <= 0.025:  # Arbitrary probability to change destination
          car.need_to_change_destination = True
          car.changed_destination = False

      # Early exit if the destination has already been changed
      if car.changed_destination:
          return

      if car.need_to_change_destination:
        if self.is_safe_to_change_lane(car) and self.check_behind(car):
            car.change_lane = True
            if car.destination_A:
                print("Changing from lane A")
                car.destination_A = False
                car.destination_B = True
                if car in self.lane_A:
                    self.lane_A.remove(car)
                self.lane_B.append(car)
            elif car.destination_B:
                print("Changing from lane B")
                car.destination_B = False
                car.destination_A = True
                if car in self.lane_B:
                    self.lane_B.remove(car)
                self.lane_A.append(car)
            car.changed_destination = True

  def early_merge(self, car, time_interval):
    if car.on_ramp:
      # Assuming merging into lane B for simplicity
      target_lane = self.lane_B
      merge_position = car.position >= 250 and car.position<=450  # Allows early merging if close to end of ramp

      if merge_position:
        # Check for safe merging conditions
        safe_to_merge_above = True
        safe_to_merge_behind = True
        car_behind=None
        car_above = None
        for lane_car in target_lane:
          if lane_car.position < car.position:
            car_behind = lane_car

            safe_distance = car_behind.speed * 2 # 2 sec rule
            if abs(lane_car.position - car.position) < safe_distance:  # Safe merging distance
             safe_to_merge_behind = False
             break


          elif lane_car.position > car.position:
            car_above = lane_car
            safe_distance = car.speed * 2 # 2 sec rule
            if abs(lane_car.position - car.position) < safe_distance:  # Safe merging distance
              safe_to_merge_above = False
              break

        can_merge = True
        closest_queue_car = None
        for ramp_car in self.ramp:
          if ramp_car.merge:
            can_merge=False
            break
          if not closest_queue_car:
            if ramp_car.speed < 3:
              closest_queue_car = ramp_car
          else:
            if ramp_car.position > car.position:
              if ramp_car.speed < 3:
                if ramp_car.position - car.position < closest_queue_car.position - car.position:
                  closest_queue_car = ramp_car

        if closest_queue_car:
          if closest_queue_car.position - car.position < 50:
            can_merge=False

        if (safe_to_merge_above and safe_to_merge_behind and can_merge) or car.merge:
          car.merge = True
          if car.y >= 85:
            # Execute the merge
            car.merged_ramp = True
            car.on_ramp = False
            self.ramp.remove(car)
            self.merged_ramp.append(car)
            target_lane.append(car)
            car.merge = False
            print(f"Car early merged at position {car.position}")
          else:
            if not safe_to_merge_behind:
                  car_behind.speed = max(car_behind.speed - 2*DECELERATION_RATE, 0)
            if not safe_to_merge_above:
                car.speed = max(car.speed - 2*DECELERATION_RATE, 0)

            car.position += car.speed * time_interval
            car.y += car.speed * self.interval/2
            #has to move to eighty-five
            car.y = min(car.y, 85)
        else:
          if car.position < 430:
            car.position += car.speed* time_interval


  def sharp_merge(self,car, time_interval):
    for car_on_ramp in self.ramp:
      if car_on_ramp != car and car_on_ramp.merge:
        car.waiting = True
        if car.waiting and car.speed < 3:
          car.wait_time += time_interval
        # self.wait_time
        return

    if car.on_ramp:
      target_lane=self.lane_B
      sharp_position=car.position>=450 or car.speed < 3

      if sharp_position:
        if not car.merge:
          car.speed = 0

        car.waiting_sharp_merge = True
        is_first_to_merge = True
        safe_to_merge = True
        car_behind = None
        # Buffer zone to slow done car behind
        for lane_car in target_lane:
          if car.position-65 < lane_car.position < car.position -10:
            lane_car.speed = max(lane_car.speed - 2*DECELERATION_RATE, 0)
        # Checking zone if they can merge or to slow down the car itself to keep up with the car in front
        for lane_car in target_lane:
          if car.position-10 < lane_car.position < car.position + 30:
            safe_to_merge = False
            if car.position < lane_car.position and car.merge:
              car.speed = max(car.speed - 2*DECELERATION_RATE, 0)
            break

        for ramp_car in self.ramp:
          if ramp_car.ramp_queue_no < car.ramp_queue_no:
            is_first_to_merge = False

        if (safe_to_merge and is_first_to_merge) or car.merge:
            car.merge = True
            car.accelerate()
            if car.y >= 85:
                car.merged_ramp = True
                car.on_ramp = False
                car.waiting = False
                self.ramp.remove(car)
                self.merged_ramp.append(car)
                target_lane.append(car)
                car.merge=False
                car.waiting_sharp_merge = False
                print(f"Car breaked/sharp merged at position {car.position}")
                if car.wait_time > 0:
                    self.waiting_list.append(car.wait_time)
                    print("waiting time", self.waiting_list)

            else:
                car.position += car.speed* time_interval
                car.y += car.speed * self.interval
                car.y = min(car.y, 85)



  def update(self):
    self.t += self.interval
    self.timesteps.append(self.t)
    self.lane_A_length.append(len(self.lane_A))
    self.lane_B_length.append(len(self.lane_B))
    self.ramp_length.append(len(self.ramp))
    self.lane_A_speed_list.append(self.lane_A_speed)
    self.lane_B_speed_list.append(self.lane_B_speed)
    # Process each car on the highway and the ramp
    for car in list(self.highway):  # Using list to avoid modification during iteration
        if car.position < 790:
          if car.keeping_safe_distance(self, self.interval):
              car.accelerate()
          else:
              car.speed = max(car.speed - DECELERATION_RATE, 0)
        else:
          car.speed = MEAN_SPEED_HIGHWAY
        car.move(self, self.interval)
        if car.position < 790:
          self.changed_destination(car)
        if car.on_ramp:
          car.move_ramp(self, self.interval)
          self.early_merge(car, self.interval)
          self.sharp_merge(car, self.interval)

  def animate(self, frame):
    self.update()
    car_objects = []
    for car in self.lane_A:
        car_objects.append(plt.Rectangle((car.position, car.y), 5.2, 3, color='blue'))
        self.axis.add_patch(car_objects[len(car_objects)-1])

    for car in self.lane_B:
        car_objects.append(plt.Rectangle((car.position, car.y), 5.2, 3, color='blue'))
        self.axis.add_patch(car_objects[len(car_objects)-1])

    for car in self.ramp:
      car_objects.append(plt.Rectangle((car.position, car.y), 5.2, 3, color='red'))  # use a different color for ramp cars
      self.axis.add_patch(car_objects[len(car_objects) - 1])

    return car_objects

  def run(self):
    global SIM_OVER

    self.anim = FuncAnimation(self.fig, self.animate, frames=1000, interval=10, blit=True)
    def close_func():
      global SIM_OVER
      SIM_OVER = True
    self.fig.canvas.mpl_connect('close_event', lambda a: close_func())

    plt.show()


if __name__ == "__main__":
    # Initialize and run simulation
    simulation = Simulation()

    car_arrival_thread_A = Thread(target=simulation.car_arrival_lane_A)
    car_arrival_thread_B = Thread(target=simulation.car_arrival_lane_B)
    car_arrival_thread_ramp = Thread(target=simulation.car_arrival_ramp)
    update_thread = Thread(target=simulation.update)

    car_arrival_thread_A.start()
    car_arrival_thread_B.start()
    car_arrival_thread_ramp.start()

    simulation.run()


    car_arrival_thread_A.join()
    car_arrival_thread_B.join()
    car_arrival_thread_ramp.join()

    simulation.run()
    if SIM_OVER == True:
        plt.figure(1)
        plt.title("Lane_Traffic vs. Time")
        plt.ylabel("Number of cars")
        plt.xlabel("t (s)")
        plt.plot(simulation.timesteps, simulation.lane_A_length, color="red", label="Lane A")
        plt.plot(simulation.timesteps, simulation.lane_B_length, color="blue", label="Lane B")
        plt.plot(simulation.timesteps, simulation.ramp_length, color="green", label="Ramp")
        plt.figure(2)
        plt.plot(simulation.timesteps, simulation.lane_A_speed_list, color='red', label="Lane A")
        plt.plot(simulation.timesteps, simulation.lane_B_speed_list, color='blue', label="Lane B")
        plt.title("Lane Speed vs. Time")
        plt.ylabel("Speed (m/s)")
        plt.xlabel("t (s)")
        plt.show()
        print("Statistics:\n")
        print("Waiting time")
        print("Mean waiting time:", np.mean(simulation.waiting_list))
        print("Standard deviation waiting time:", np.std(simulation.waiting_list))
        print("Minimum waiting time (> 0)", np.min(simulation.waiting_list))
        print("25 percentile waiting time", np.percentile(simulation.waiting_list, 25))
        print("Median waiting time", np.percentile(simulation.waiting_list, 50))
        print("75 percentile waiting time", np.percentile(simulation.waiting_list, 75))
        print("Maximum waiting time", np.max(simulation.waiting_list))
        print("\nTraffic speed A")
        print("Mean traffic speed A:", np.mean(simulation.lane_A_speed_list))
        print("Standard deviation traffic speed A", np.std(simulation.lane_A_speed_list))
        print("Minimum traffic speed A", np.min(simulation.lane_A_speed_list))
        print("25 percentile traffic speed A", np.percentile(simulation.lane_A_speed_list, 25))
        print("Median traffic speed A", np.percentile(simulation.lane_A_speed_list, 50))
        print("75 percentile traffic speed A", np.percentile(simulation.lane_A_speed_list, 75))
        print("Maximum traffic speed A", np.max(simulation.lane_A_speed_list))
        print("\nTraffic speed B")
        print("Mean traffic speed B:", np.mean(simulation.lane_B_speed_list))
        print("Standard deviation traffic speed B", np.std(simulation.lane_B_speed_list))
        print("Minimum traffic speed B", np.min(simulation.lane_B_speed_list))
        print("25 percentile traffic speed B", np.percentile(simulation.lane_B_speed_list, 25))
        print("Median traffic speed B", np.percentile(simulation.lane_B_speed_list, 50))
        print("75 percentile traffic speed B", np.percentile(simulation.lane_B_speed_list, 75))
        print("Maximum traffic speed B", np.max(simulation.lane_B_speed_list))