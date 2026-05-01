from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the GhostPilot agent stack."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    llm_provider = LaunchConfiguration('llm_provider')
    llm_model = LaunchConfiguration('llm_model')
    llm_endpoint = LaunchConfiguration('llm_endpoint')
    use_examples = LaunchConfiguration('use_examples')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
            DeclareLaunchArgument('llm_provider', default_value='ollama', description='LLM provider'),
            DeclareLaunchArgument('llm_model', default_value='llama3', description='LLM model name'),
            DeclareLaunchArgument('llm_endpoint', default_value='http://localhost:11434', description='LLM endpoint'),
            DeclareLaunchArgument('use_examples', default_value='true', description='Enable few-shot examples'),
            Node(
                package='ghostpilot_agent',
                executable='mission_parser',
                name='mission_parser',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'llm_provider': llm_provider,
                        'llm_model': llm_model,
                        'llm_endpoint': llm_endpoint,
                        'use_examples': use_examples,
                    }
                ],
            ),
            Node(
                package='ghostpilot_agent',
                executable='executor',
                name='mission_executor',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )
