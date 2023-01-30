"""Module to execute the actions asynchornously using dependency graph."""
import asyncio
import networkx as nx


class Executor:
    """Class to execute the actions asynchornously using dependency graph."""

    def __init__(self, graph):
        """Initialize the executor with the dependency graph."""
        self.graph = graph
        self.loop = asyncio.get_event_loop()
        self.tasks = []
        self._create_tasks()

    def _create_tasks(self):
        """Create tasks for each action."""
        for node in self.graph.nodes:
            self.tasks.append(self.loop.create_task(node))

    async def execute(self):
        """Execute the actions."""
        await asyncio.gather(*self.tasks)