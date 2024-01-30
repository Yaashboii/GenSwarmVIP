from typing import Any, Iterable, Type, Union, Set, final
from pydantic import BaseModel, ConfigDict, SerializeAsAny, Field

from modules.utils.logger import setup_logger, LoggerLevel
from modules.llm.gpt import GPT
from modules.actions.action import Action
from modules.utils.common import any_to_str
from modules.framework.message import MessageQueue, Message


PREFIX_TEMPLATE = """You are a {profile}, named {name}, your goal is {goal}. """
CONSTRAINT_TEMPLATE = "the constraint is {constraints}. "

class RoleContext(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    msg_buffer: MessageQueue = Field(
        default_factory=MessageQueue, exclude=True
    )
    framework: "Framework" = Field(default=None, exclude=True)

class Role(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    name: str = ""
    profile: str = ""
    goal: str = ""
    constraints: str = ""
    subscription: set[str] = set()
    llm: GPT = Field(default=GPT, exclude=True)
    actions: dict[str, SerializeAsAny[Action]] = Field(default={}, validate_default=True)
    watch: set[str] = Field(default=set)    
    next_action: Action = Field(default=None, exclude=True)
    rc: RoleContext = Field(default_factory=RoleContext)

    __hash__ = object.__hash__  # support Role as hashable type

    def __init__(self, **data: dict):
        super(Role, self).__init__(**data)
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)

    def put_message(self, message):
        """Place the message into the Role object's private message buffer."""
        if message:
          self.rc.msg_buffer.push(message)
    
    def set_framework(self, framework):
        self.rc.framework = framework
        framework.members.append(self)
    
    @final
    async def run(self):
        self._logger.info("running %s", self.__class__.__name__)
        news = await self._observe()
        if not len(news):
            self._logger.debug("no news. waiting.")
            return
        for msg in news: # TODO: not sure the result
          rsp = await self._react(msg)
          self.next_action = None
          if rsp and self.rc.framework:
            self.rc.framework.publish_message(rsp)

    def _init_actions(self, actions):
        for action in actions:
            if not isinstance(action, Action):
                tmp = action(name="", llm=self.llm)
            else:
                tmp = action
            tmp.set_prefix(self._get_prefix())
            self.actions[tmp.name] = tmp

    def _get_prefix(self) -> str:
        prefix = PREFIX_TEMPLATE.format(**{
            "profile": self.profile, 
            "name": self.name, 
            "goal": self.goal
        })
        if self.constraints:
            prefix += CONSTRAINT_TEMPLATE.format(**{
                "constraints": self.constraints
            })
        return prefix
  
    def _watch(self, actions: Union[Iterable[Type[Action]], Iterable[Action]]):
        self.watch = {any_to_str(t) for t in actions}

    @final
    async def _observe(self):
        news = self.rc.msg_buffer.pop_all()
        news = [n for n in news if (n.cause_by in self.watch)]
        news_text = [f"{i.role}: {i.content[:20]}..." for i in news]
        if news_text:
            self._logger.debug(f"{self.name} observed: {news_text}")
        return news

    @final
    async def _react(self, msg) -> Message:
        rsp = Message(content="No actions taken yet", cause_by=Action)  # will be overwritten after Role _act
        await self._think(msg)
        self._logger.debug("next action is %s", self.next_action)
        if self.next_action is None:
            return None
        # act
        self._logger.debug(f"will do {self.next_action}")
        rsp = await self._act(msg)
        return rsp  # return output from the last action
    
    async def _think(self, msg):
        if len(self.actions) == 1:
          _, action = next(iter(self.actions.items()))
          self.next_action = action
    
    async def _act(self, msg) -> Message:
        self._logger.debug(
            f"{self.name}: to do {self.next_action}({self.next_action.name})")
        response = await self.next_action.run(msg.content)
        if isinstance(response, Message):
            msg = response
        else:
            msg = Message(content=response, role=self.profile,
                          cause_by=self.next_action, sent_from=self)
        return msg


if __name__ == "__main__":
    """TODO: add test"""  