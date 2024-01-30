import asyncio

import typer

app = typer.Typer(add_completion=False)

@app.command()
def startup(
    query: str = typer.Argument(..., help="Your query.'"),
    n_round: int = typer.Option(default=5, help="Number of rounds for the simulation."),

):
    from modules.framework.framework import Framework
    from modules.roles.actor import Actor
    from modules.roles.critic import Critic
    framework = Framework()
    framework.add_roles([
        Actor(),
        Critic()
    ])

    asyncio.run(framework.run(query=query, n_round=n_round))


if __name__ == "__main__":
    app()