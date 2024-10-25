import unittest
import numpy as np
from unittest.mock import patch
from Box2D import b2Vec2
from modules.deployment.entity.base_entity import Entity
from modules.deployment.engine.box2d_engine import (
    Box2DEngine,
)  # Replace 'your_module' with the module containing Box2DEngine


class MockEntity(Entity):
    def __init__(
        self,
        entity_id: int,
        initial_position: list[float] | tuple | np.ndarray = np.zeros(2),
        size: list[float] | tuple | np.ndarray | float = 1.0,
        color: str | tuple = "blue",
        collision: bool = False,
        movable: bool = False,
        max_speed: float = 1.0,
        mass: float = 1.0,
        density: float = 0.1,
        shape: str = "circle",
    ):
        super().__init__(
            entity_id,
            initial_position,
            size,
            color,
            collision,
            movable,
            max_speed,
            mass,
            density,
            shape,
        )


class TestBox2DEngine(unittest.TestCase):
    def setUp(self):
        self.engine = Box2DEngine(
            gravity=(0, -10)
        )  # Initialize the engine with gravity
        self.entity = MockEntity(
            1, initial_position=[0, 0], size=1.0, color="blue", movable=True
        )
        self.engine.add_entity(self.entity)

    @patch.object(Box2DEngine, "apply_force")
    def test_apply_force(self, mock_apply_force):
        # Apply a force to an entity and verify
        force = np.array([10.0, 0.0])
        self.engine.apply_force(self.entity.id, force)
        mock_apply_force.assert_called_once_with(self.entity.id, force)

    @patch.object(Box2DEngine, "control_velocity")
    def test_control_velocity(self, mock_control_velocity):
        # Test control_velocity by setting desired_velocity
        desired_velocity = np.array([5.0, 0.0])
        self.engine.control_velocity(self.entity.id, desired_velocity, dt=0.1)
        mock_control_velocity.assert_called_once_with(
            self.entity.id, desired_velocity, dt=0.1
        )

    def test_step_updates_position_and_velocity(self):
        # Test the step method updates positions and velocities
        initial_position, _ = self.engine.get_entity_state(self.entity.id)
        self.engine.step(1.0)  # Step by 1 second
        updated_position, updated_velocity = self.engine.get_entity_state(
            self.entity.id
        )
        self.assertFalse(
            np.array_equal(initial_position, updated_position),
            "Position should update after step",
        )

    def test_add_entity(self):
        # Test adding an entity creates a Box2D body
        self.assertIn(self.entity.id, self.engine.bodies)
        body = self.engine.bodies[self.entity.id]
        self.assertIsNotNone(body)
        self.assertEqual(body.position, b2Vec2(self.entity.position))

    def test_remove_entity(self):
        # Test removing an entity deletes it from Box2D world
        self.engine.remove_entity(self.entity.id)
        self.assertNotIn(self.entity.id, self.engine.bodies)

    def test_add_joint(self):
        # Test adding a joint between two entities
        entity2 = MockEntity(2, initial_position=[5, 0], size=1.0, color="red")
        self.engine.add_entity(entity2)
        self.engine.add_joint(self.entity.id, entity2.id, distance=5.0)
        self.assertIn((self.entity.id, entity2.id), self.engine.joints)

    def test_remove_joint(self):
        # Test removing a joint between two entities
        entity2 = MockEntity(2, initial_position=[5, 0], size=1.0, color="red")
        self.engine.add_entity(entity2)
        self.engine.add_joint(self.entity.id, entity2.id, distance=5.0)
        self.engine.remove_joint(self.entity.id, entity2.id)
        self.assertNotIn((self.entity.id, entity2.id), self.engine.joints)


if __name__ == "__main__":
    unittest.main()
