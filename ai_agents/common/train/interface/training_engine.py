from abc import ABC, abstractmethod

class TrainingEngine(ABC):
    @abstractmethod
    def train(self) -> None:
        pass

    @abstractmethod
    def test(self) -> None:
        pass