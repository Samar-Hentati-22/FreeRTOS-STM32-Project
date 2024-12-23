# FreeRTOS-STM32-Project 
This project was developed as part of a FreeRTOS workshop, where I learned the fundamentals of real-time operating systems and applied them to create a complete STM32-based application.
## What I learned
These are the key concepts to know about the fundamentals of real-time operating systems(RTOS): 

### 1. Difference Between Applications with an OS and Without an OS
- **Bare-Metal Systems:**
  - These systems operate without an operating system, meaning the application code directly interacts with the hardware.
  - Tasks are executed sequentially, which makes timing-critical applications difficult to manage as complexity grows.
  - **Example:** Polling-based systems or simple embedded systems with a single control loop.
- **With an OS:**
  Using an RTOS allows us to manage complexity while ensuring the system responds to real-time events. 
An RTOS excels at:
- Task prioritization: Ensuring that higher-priority tasks preempt lower-priority ones.
- Timing precision: Managing critical deadlines efficiently.

### 2. When to migrate from Bare Metal to Embedded OS: 
- **Bare Metal:** When the system is simple and timing requirements are predictable, such as controlling LEDs or managing a basic sensor.
- **Embedded OS (FreeRTOS):**
  - When the system is more complex and requires multitasking.
  - If tasks need to respond to events with strict timing constraints.
  - When inter-task communication and resource sharing are needed.

### 3. What is a Real-Time System?
it's any system designed to provide a **deterministic response** to specific events within a defined time frame (called deadline) 
  - **Hard Real-Time Systems:** Missing deadlines can lead to system failure (medical devices, aerospace systems).
  - **Soft Real-Time Systems:** Deadlines are flexible and occasional misses are tolerable (multimedia streaming).

### 4. What is FreeRTOS?
FreeRTOS is an open-source real-time operating system specifically designed for embedded systems. It provides:  
- A **lightweight kernel** capable of running on microcontrollers with limited resources.  
- Support for **multitasking**, enabling the execution of multiple tasks efficiently.  
- **APIs for synchronization** (semaphores, mutexes) and **communication** (queues).  
- High portability, supporting a wide range of microcontroller architectures.  
#### **CMSIS-RTOS Integration**
In this project, we used **CMSIS-RTOS**, which is a standard interface provided by ARM for integrating RTOS functionality with Cortex-M processors. CMSIS-RTOS acts as a wrapper around FreeRTOS, providing:  
- **Abstraction**: Simplifying RTOS API usage for developers.  
- **Portability**: Allowing easier migration between different RTOS implementations.  
- **Compatibility**: Integrating seamlessly with STM32CubeIDE for our project.
### 5. Core Concepts in FreeRTOS
#### Tasks
- Tasks are the smallest units of execution in FreeRTOS.
- **Task States:**
  - **Ready:** Task is ready to be executed but is not currently executing because a different task with equal or higher priority is running.
  - **Running:** Task is actually running( only one can be in this state at the moment).
  - **Blocked:** Task is waiting for a either a temporal or external event.
  - **Suspended:** Task is not available for scheduling, but still being kept in memory.
  - **Deleted:** Task is no longer part of the system.
#### Scheduler
It's an algorithm that decides which task based on priority( It selects a task of the list of Tasks that are READY) 
- FreeRTOS uses a **preemptive scheduler**, meaning higher-priority tasks can interrupt lower-priority ones.
- This ensures critical tasks are executed on time.
#### Queues 
 Queues are pipes that allow tasks to exchange data, they are used for **inter-task communication**
- The default queue is behaving as FIFO (First In-First Out) , we can also redefine it to perform as LIFO(Last In First Out)
- Within CMSIS-RTOS API there are two types of queues:
  -**Message**: where we can send only ineger type data or a pointer
  -**Mail**: where we can send memory blocks
- **Example:** Sending sensor data from a data acquisition task to a logging task.
#### Semaphores
-They are used to synchronize tasks with other events in the systems or to protect shared resources.
We can either: 

 -**Turn On** Semaphore: means to give a semaphore and can be done from another task or from an interrupt subroutine.
 
 -**Turn Off** Semaphore: means to Take the semaphore and can be done from the task.
 
-Types of Semaphores: 

 -**Binary** Simple on/off mechanism.
 -**Counting**: counts multiple give and multiple take.
 -**Mutex**-Mutual Exclusion type Semaphore:they include a **priority inheritance mechanism** to prevent priority inversion.
   - **Example:** Ensuring a single task accesses a shared memory region at a time.
     
## Helpful Resources
If you're new to FreeRTOS, I recommend checking out this excellent [YouTube series on FreeRTOS fundamentals](<https://www.youtube.com/playlist?list=PLEBQazB0HUyQ4hAPU1cJED6t3DU0h34bz>). It provides a clear and comprehensive introduction to the concepts I talked about.

### Implementation Details  
Below, you'll find the specific implementation of this project, demonstrating how the features and concepts explained above were realized using FreeRTOS and the STM32 platform.
























