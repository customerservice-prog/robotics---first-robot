from abc import ABC, abstractmethod

from app.models import RobotStatus


class RobotHardware(ABC):
    @abstractmethod
    def drive(self, left: int, right: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def status(self) -> RobotStatus:
        raise NotImplementedError

    def reset_encoders(self) -> None:
        """Optional encoder reset hook. Odometry can also re-baseline without resetting hardware."""
        pass

    def close(self) -> None:
        pass
