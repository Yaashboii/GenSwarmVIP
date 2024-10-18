## Using Docker

### Installing Docker

If you haven't installed Docker yet, follow the official documentation to install Docker: [Get Docker](https://docs.docker.com/get-docker/)

### Installing Docker Compose

If Docker Compose isn't installed yet, follow the official Docker documentation to install Docker Compose: [Install Docker Compose](https://docs.docker.com/compose/install/).

### Building Docker Images

To build the Docker images defined in your `docker-compose.yml` file, follow these steps:

1. **Navigate to the directory where the `docker-compose.yml` file is located**. In this case, switch to the `/project_path/docker` directory using the following command:

    ```bash
    cd /project_path/docker
    ```

2. **Build all Docker images** by running the following command:

    ```bash
    docker-compose build
    ```

    This will trigger the build process for all services defined in the `docker-compose.yml` file:

    - **`base`**: Builds using `base.Dockerfile` in the current directory (`.`).
    - **`runtime-base`**: Extends the `base` service and builds using `runtime.Dockerfile` located one directory above (`..`).
    - **`unittest`**: Extends the `runtime-base` service and runs the unit tests using `python -m unittest discover` after building.
    - **`deploy`**: Extends the `base` service and builds using `deploy.Dockerfile` from the parent directory. It uses Ansible to run deployment scripts.

    Each image will be tagged according to the `image` field in the `docker-compose.yml` file, such as `huabench/code-llm:base`, `huabench/code-llm:runtime`, and `huabench/code-llm:deploy`.

### Running the Containers

After building the images, you can start the containers:

- To start the services, run:

    ```bash
    docker-compose up
    ```

    This will bring up all the services defined in the `docker-compose.yml` file in the foreground.

- To start the containers in the background (detached mode), run:

    ```bash
    docker-compose up -d
    ```

- **Run Unit Tests**:

    The `unittest` service is configured to run unit tests. To run the tests, use:

    ```bash
    docker-compose run unittest
    ```

- **Run the Deployment Script**:

    The `deploy` service is configured to run an Ansible playbook. To execute the deployment, use:

    ```bash
    docker-compose run deploy
    ```
- **Run the Runtime Environment**:

    The `runtime-base` service starts a container with the project's necessary environment and code. To run this service, use:

    ```bash
    docker-compose run runtime-base
    ```

### Stopping the Services

To stop and remove the running containers, use:

```bash
docker-compose down
