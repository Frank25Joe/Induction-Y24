# Induction-Y24

Make a fork of this repo and create a branch in the fork with the following name: `[firstname]_[lastname_initial]`  
Example: `atulya_s`

## Task 1

### OverTheWire: Bandit  

Complete levels 0 to 15 of the [Bandit wargame](http://overthewire.org/wargames/bandit/).

- Create a markdown file named `otw.md`.
- For each level (0 to 15), write a short 2–3 line explanation covering:
  - What the level asked for
  - How you solved it (commands, logic, etc.)

- Commit this file to your branch.

## Task 2
### Netflix Inventory Management

You got your dream job in **MAANG**. To give other employees a run for their money, you have to come up with an **OOPS-based content rental client**.

---

#### High-Level Overview

Create a Netflix-style inventory management system using Object-Oriented Programming (OOPS). The system should manage a collection of movies, TV shows, and users, allowing them to browse, rent, and return content.
NOTE: Your Program should retain its memory of whatever data has been fed by the user

---

#### Login and Account Creation

1. The system should be accessed via a **CLI (Command Line Interface)**.
2. It should have options for:
   - **User Login / Sign-Up**
   - **Admin Login**
3. Users choose their own username. The system should ensure **uniqueness of usernames**.

---

#### User Account Features

1. Browse content by **category** (Movies or TV Shows) or **genre**.
2. **Search** content by **title** or **genre**.
3. **Rent** movies or TV shows.
4. **Return** rented content.
5. View currently rented items along with:
   - **Date rented**
   - **Last date of return**
6. View **purchased** items.
7. Check total **charges due**.

---

####  Admin Account Features

1. **Add** new movies and TV shows.
2. **Remove** existing movies and TV shows.
3. **Check** charges due for any user.

---

#### Content Structure

##### Common Attributes:
- **Title**
- **Genre**
- **Rating**
- `is_rented`
- `is_purchased`

---

##### Movie:
- **Duration**
- **Rent cost**
- **Purchase cost**

---

##### TV Show:
- **Seasons**
- **Episodes per season**
- **Per season rent cost**
- **Per season purchase cost**
---

#### Submission

- Fork the GitHub repository.
- Create a new branch with your name:  
  Format — `[firstname]_[lastname_initial]` (e.g., `atulya_s`)
- Submit a **pull request** by:  
   **11:59:59 PM, 13th May 2025**
- For any doubts, open an issue on the main repo.
- The task should be contained in a directory structured as Task2/{[firstname]_[lastname_initial]}/
- Also your code should be written in C++
- Your Code should have a Dockerfile along with it and we should be able to run it through docker as well
---

#### Suggestions

- Do **not** use GPT or other AI tools as we might very well ask you to explain your code as well as your thought process.
- Use your **own logic** and write clean, structured code.
- If you think you can make the program with alternative methods like MongoDB or SQL then you are free to do so.(But then your code should still run inside docker satisfying all the requirements)
- Feel free to get **creative** with features, but stick to the core requirements.
- Hint: For storage think of each object as a file.
---
# Task 3
# ROS 2 Project: Printer Server with Print Queue and Job Status Publisher

# 🧠 Objective

Create a ROS 2-based Printer Server system where:

* Users can submit print jobs via a ROS 2 service.
* Jobs are queued and processed sequentially.
* Status of each job is published to a ROS 2 topic.
* The entire system runs inside a Docker container.

This task helps in understanding ROS 2 services, publishers/subscribers, custom messages, managing node logic, and containerization.

---

## 🧩 Components Overview

### 1. **Print Job Service** (`PrintJob.srv`)

Custom service to submit print jobs.

**Definition:**

```srv
string document_name
---
bool accepted
```

---

### 2. **Printer Node**

Responsible for:

* Receiving print job service calls.
* Queueing jobs internally.
* Processing one job at a time (simulated delay).
* Publishing status messages for each job.

**Publishes:** `/print_status` (`std_msgs/String`)

**Service Server:** `/print_job` (uses `PrintJob.srv`)

**Status Examples:**

* "Job Queued: file1.pdf"
* "Started printing: file1.pdf"
* "Completed printing: file1.pdf"

---

### 3. **Client Node**

Create a dedicated ROS 2 node that sends print job requests to the server by calling the `/print_job` service.

**Responsibilities:**

* Accept document names as input from the user or argument.
* Send a request using the `PrintJob` service.
* Optionally, subscribe to `/print_status` to track progress.

---

## 📁 Suggested Package Structure

```
printer_system/
├── printer_server/
│   ├── printer_node.py           # Contains queue and service logic
│   └── CMakeLists.txt
├── printer_client/
│   ├── client_node.py            # Node to send job requests
│   └── CMakeLists.txt
├── srv/
│   └── PrintJob.srv              # Custom service definition
├── launch/
│   └── printer_launch.py         # Launch all nodes together (optional)
└── package.xml
```

---

## 🚀 Launch Instructions (Without Docker)

1. **Build the workspace**

```bash
colcon build
source install/setup.bash
```

2. **Run the Printer Node**

```bash
ros2 run printer_server printer_node
```

3. **Run the Client Node**

```bash
ros2 run printer_client client_node --ros-args -p document_name:=myfile.pdf
```

4. **Monitor Status Topic**

```bash
ros2 topic echo /print_status
```

---

## 🧪 Example Outputs

### ✅ 1. Service Call: `/print_job`

```bash
ros2 service call /print_job printer_server/srv/PrintJob "{document_name: 'example.pdf'}"
```

**Output:**

```
requester: making request: printer_server.srv.PrintJob_Request(document_name='example.pdf')

response:
  accepted: True
```

### ✅ 2. Printer Node Terminal Output

```
[INFO] [printer_node]: Received print job request: example.pdf
[INFO] [printer_node]: Job Queued: example.pdf
[INFO] [printer_node]: Started printing: example.pdf
[INFO] [printer_node]: Printing...
[INFO] [printer_node]: Printing...
[INFO] [printer_node]: Completed printing: example.pdf
```

### ✅ 3. Topic Echo: `/print_status`

```bash
ros2 topic echo /print_status
```

**Output:**

```yaml
data: "Job Queued: example.pdf"
---
data: "Started printing: example.pdf"
---
data: "Completed printing: example.pdf"
```

### ✅ 4. Multiple Job Requests

```bash
ros2 service call /print_job printer_server/srv/PrintJob "{document_name: 'doc1.pdf'}"
ros2 service call /print_job printer_server/srv/PrintJob "{document_name: 'doc2.pdf'}"
ros2 service call /print_job printer_server/srv/PrintJob "{document_name: 'doc3.pdf'}"
```

**Printer Node Output:**

```
[INFO] [printer_node]: Received print job request: doc1.pdf
[INFO] [printer_node]: Job Queued: doc1.pdf
[INFO] [printer_node]: Received print job request: doc2.pdf
[INFO] [printer_node]: Job Queued: doc2.pdf
[INFO] [printer_node]: Received print job request: doc3.pdf
[INFO] [printer_node]: Job Queued: doc3.pdf
[INFO] [printer_node]: Started printing: doc1.pdf
[INFO] [printer_node]: Completed printing: doc1.pdf
[INFO] [printer_node]: Started printing: doc2.pdf
[INFO] [printer_node]: Completed printing: doc2.pdf
[INFO] [printer_node]: Started printing: doc3.pdf
[INFO] [printer_node]: Completed printing: doc3.pdf
```

---

## 🔧 Optional Extensions

* Add priority levels to jobs.
* Support for job cancellation.
* Save logs of completed jobs to a file.
* Create a simple frontend using `rqt`.

---

## 🎓 Learning Outcomes

* ROS 2 service creation and usage
* Topic publishing and subscribing
* Managing internal state (queues) in nodes
* Designing ROS packages with modular architecture
* Containerizing ROS 2 projects for deployment
## Note: You need to provide multiple docker files or single if you want to seperate server and client (optional) as you wish along with compose.yaml or bash script
---

Happy ROSing! 🖨️🤖
