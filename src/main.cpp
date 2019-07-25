#include "cli_application.hpp"
#include "inspect.hpp"

int main(int argc, char const* const* argv)
{
    cli_application app(
        "CSE CLI",
        "0.1.0",
        "cse",
        "Command-line interface to the Core Simulation Environment",
        "The Core Simulation Environment is free and open-source software for running distributed co-simulations.");
    app.add_subcommand(std::make_unique<inspect_subcommand>());
    return app.run(argc, argv);
}
