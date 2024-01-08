import logging

from modules.framework.actor import Actor
from modules.framework.critic import CodeErrorCritic, TheoreticalCollisionCritic
from modules.framework.agent import Robot


class Core():
    def __init__(self) -> None:
        self._actor = Actor()
        self._robot = Robot()
        self._code_critic = CodeErrorCritic()
        self._collision_critic = TheoreticalCollisionCritic()
        self._code_critics = [self._code_critic, self._collision_critic]
        self._max_error_count = 3

        self._logger = logging.getLogger("Core")
        self._logger.setLevel(logging.INFO)

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
                
        self._robot.execute_code(code)
        self._logger.info("code finished")

        # TODO: environment feedback

if __name__ == '__main__':
    log_format = '\033[92m [%(asctime)s] \033[0m \033[97m%(message)s\033[0m'

    # log_format = '[%(asctime)s Core] %(message)s'
    logging.basicConfig(level=logging.CRITICAL, format=log_format)

    core = Core()
    core.step("计算两数之差")