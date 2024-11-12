from rich.panel import Panel
from rich import print
from rich.syntax import Syntax


def rich_print(title, content, subtitle=""):
    panel = Panel(
        content,
        title=f"[bold cyan]{title}[/bold cyan]",
        border_style="cyan",  # Border color
        subtitle=f"[bold cyan]{subtitle}[/bold cyan]",
    )
    print(panel)


def rich_code_print(title, code, subtitle=""):
    if subtitle:
        subtitle = f"[bold cyan]{subtitle}.py[/bold cyan]"
    syntax = Syntax(code, "python", theme="monokai", line_numbers=True)
    panel = Panel(
        syntax,
        title=f"[bold cyan]{title}[/bold cyan]",
        border_style="cyan",
        subtitle=subtitle,
    )
    print(panel)
