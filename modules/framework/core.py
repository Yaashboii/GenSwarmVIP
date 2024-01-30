from modules.utils.logger import setup_logger
from modules.roles.actor import Actor
from modules.framework.critic import CodeErrorCritic, TheoreticalCollisionCritic
# from modules.framework.agent import Robot


class Core():
    def __init__(self) -> None:
        self._actor = Actor()
        # self._robot = Robot()
        self._code_critic = CodeErrorCritic()
        self._collision_critic = TheoreticalCollisionCritic()
        self._code_critics = [self._code_critic, self._collision_critic]
        self._max_error_count = 3

        self._logger = setup_logger("Core", "INFO")

    def step(self, query: str):
        self._logger.info(f"Your query is: {query}")
        self._logger.info("Generating code...")
        code = self._actor.generate_code(query)
        self._logger.info(f"Code generated: \n{code}")

        for ind, critic in enumerate(self._code_critics):
            # Iterate through error correction attempts
            for _ in range(self._max_error_count):
                is_passed, suggection = critic.evaluate(code)
                self._logger.info(f"Critic {critic.__class__.__name__}, Round {ind + 1}: {is_passed}, suggestion is {suggection}")

                if not is_passed:
                    code = self._actor.modify_code(code, suggection)
                    self._logger.info(f"Code not pass, regenerated code: \n{code}")

                else:
                    break

            if not is_passed:
                self._logger.critical("code is not passed")
                break
                
        # self._robot.execute_code(code)
        self._logger.info("code finished")

        # TODO: environment feedback

if __name__ == '__main__':
    core = Core()

    query = "write a function vel_x, vel_y = move(target_x, target_y)"
    core.step(query)