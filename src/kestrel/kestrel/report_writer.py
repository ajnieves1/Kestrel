# Collect mission data and write an inspection report when the mission lands
import base64
import json
import math
import os
import shutil
import time

import anthropic
import openai
import rclpy
from google import genai
from google.genai import types
from kestrel_msgs.msg import DefectEvent, HealthAlert
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

SYSTEM_PROMPT = (
    'You are an inspection engineer. Write the findings section of a drone '
    'inspection report for a transmission pylon. For each defect photo, '
    'describe what is visible, state the recorded position, and give a '
    'severity guess. Plain language, markdown, no preamble.')

PROVIDER_DEFAULT_MODELS = {
    'claude': 'claude-sonnet-5',
    'openai': 'gpt-5',
    'gemini': 'gemini-2.5-flash',
}

PROVIDER_KEY_ENV_VARS = {
    'claude': 'ANTHROPIC_API_KEY',
    'openai': 'OPENAI_API_KEY',
    'gemini': 'GEMINI_API_KEY',
}

# Defects within this many meters of a previous defect count as the same one
MATCH_RADIUS_METERS = 3.0


# Collect mission data and write an inspection report when the mission lands
class ReportWriter(Node):
    # Subscribe to defect events and mission state
    def __init__(self):
        super().__init__('report_writer')

        self.declare_parameter('llm_provider', 'claude')
        self.declare_parameter('llm_model', '')
        self.declare_parameter('site_name', 'pylon')

        self.llm_provider = self.get_parameter('llm_provider').value
        self.llm_model = self.get_parameter('llm_model').value
        self.site_name = self.get_parameter('site_name').value

        self.defect_events = []
        self.health_alerts = []
        self.takeoff_time = None
        self.landed_reported = False

        self.create_subscription(
            DefectEvent, '/kestrel/defect_events', self.on_defect_event,
            qos_profile_sensor_data)
        self.create_subscription(
            HealthAlert, '/kestrel/health_alerts', self.on_health_alert,
            qos_profile_sensor_data)
        self.create_subscription(
            String, '/kestrel/mission_state', self.on_mission_state,
            qos_profile_sensor_data)

    # Store each defect event
    def on_defect_event(self, defect_event_message):
        self.defect_events.append(defect_event_message)

    # Store each health alert
    def on_health_alert(self, health_alert_message):
        self.health_alerts.append(health_alert_message)

    # Track mission timing and trigger the report on landing
    def on_mission_state(self, state_message):
        if state_message.data == 'TAKEOFF' and self.takeoff_time is None:
            self.takeoff_time = time.time()
        if state_message.data == 'LANDED' and not self.landed_reported:
            self.landed_reported = True
            self.write_report()

    # Ask the configured LLM for a findings section, None on any failure
    def request_findings(self, defect_summaries):
        model_name = self.llm_model or PROVIDER_DEFAULT_MODELS.get(self.llm_provider)

        if self.llm_provider not in PROVIDER_DEFAULT_MODELS:
            self.get_logger().warn(
                f'unknown llm_provider {self.llm_provider}, valid values are '
                'claude, openai, gemini')
            return None

        key_env_var = PROVIDER_KEY_ENV_VARS[self.llm_provider]
        if not os.environ.get(key_env_var):
            self.get_logger().warn(f'{key_env_var} is not set, skipping findings')
            return None

        prompt_parts = self.build_prompt_parts(defect_summaries)
        try:
            if self.llm_provider == 'claude':
                return self.request_findings_claude(prompt_parts, model_name)
            if self.llm_provider == 'openai':
                return self.request_findings_openai(prompt_parts, model_name)
            return self.request_findings_gemini(prompt_parts, model_name)
        except Exception as request_error:
            self.get_logger().warn(f'llm request failed: {request_error}')
            return None

    # Build the intro text and the per defect text plus photo path pairs
    def build_prompt_parts(self, defect_summaries):
        intro = (
            f'Mission found {len(defect_summaries)} defects on a transmission '
            'pylon inspection. Write the findings section described in your '
            'instructions.')

        defect_parts = []
        for summary in defect_summaries:
            position = summary['world_position']
            stats_text = (
                f"Defect {summary['label']}, confidence "
                f"{summary['confidence']:.2f}, position north "
                f"{position.x:.2f} east {position.y:.2f} altitude "
                f"{position.z:.2f}")
            defect_parts.append((stats_text, summary['photo_path']))

        return intro, defect_parts

    # Send the prompt and photos to the Claude API and return markdown
    def request_findings_claude(self, prompt_parts, model_name):
        intro, defect_parts = prompt_parts
        client = anthropic.Anthropic()

        content = [{'type': 'text', 'text': intro}]
        for stats_text, photo_path in defect_parts:
            content.append({'type': 'text', 'text': stats_text})
            content.append({
                'type': 'image',
                'source': {
                    'type': 'base64',
                    'media_type': 'image/jpeg',
                    'data': self.encode_photo_base64(photo_path),
                },
            })

        response = client.messages.create(
            model=model_name, max_tokens=1500, system=SYSTEM_PROMPT,
            messages=[{'role': 'user', 'content': content}])

        return ''.join(
            block.text for block in response.content if block.type == 'text')

    # Send the prompt and photos to the OpenAI API and return markdown
    def request_findings_openai(self, prompt_parts, model_name):
        intro, defect_parts = prompt_parts
        client = openai.OpenAI()

        content = [{'type': 'text', 'text': intro}]
        for stats_text, photo_path in defect_parts:
            content.append({'type': 'text', 'text': stats_text})
            encoded_photo = self.encode_photo_base64(photo_path)
            content.append({
                'type': 'image_url',
                'image_url': {'url': f'data:image/jpeg;base64,{encoded_photo}'},
            })

        response = client.chat.completions.create(
            model=model_name, max_completion_tokens=4000,
            messages=[
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': content},
            ])

        return response.choices[0].message.content

    # Send the prompt and photos to the Gemini API and return markdown
    def request_findings_gemini(self, prompt_parts, model_name):
        intro, defect_parts = prompt_parts
        client = genai.Client()

        contents = [intro]
        for stats_text, photo_path in defect_parts:
            contents.append(stats_text)
            with open(photo_path, 'rb') as photo_file:
                photo_bytes = photo_file.read()
            contents.append(types.Part.from_bytes(data=photo_bytes, mime_type='image/jpeg'))

        response = client.models.generate_content(
            model=model_name, contents=contents,
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_PROMPT, max_output_tokens=1500))

        return response.text

    # Read a saved defect photo and return it as a base64 string
    def encode_photo_base64(self, photo_path):
        with open(photo_path, 'rb') as photo_file:
            return base64.b64encode(photo_file.read()).decode('utf-8')

    # Write the machine readable defect record next to the report
    def write_defects_json(self, report_directory, defect_summaries):
        duration_seconds = time.time() - self.takeoff_time if self.takeoff_time else 0.0
        record = {
            'timestamp': os.path.basename(report_directory),
            'site': self.site_name,
            'duration_seconds': duration_seconds,
            'defects': [
                {
                    'label': summary['label'],
                    'confidence': summary['confidence'],
                    'north': summary['world_position'].x,
                    'east': summary['world_position'].y,
                    'altitude': summary['world_position'].z,
                    'image': os.path.basename(summary['photo_path']),
                }
                for summary in defect_summaries
            ],
        }
        with open(os.path.join(report_directory, 'defects.json'), 'w') as defects_file:
            json.dump(record, defects_file, indent=2)

    # Load the newest previous defects.json for this site, None when absent
    def load_previous_defects(self):
        if not os.path.isdir('reports'):
            return None

        matching_records = []
        for entry_name in sorted(os.listdir('reports')):
            defects_path = os.path.join('reports', entry_name, 'defects.json')
            if not os.path.isfile(defects_path):
                continue
            with open(defects_path) as defects_file:
                record = json.load(defects_file)
            if record.get('site') == self.site_name:
                matching_records.append(record)

        return matching_records[-1] if matching_records else None

    # Classify defects as new, persistent, or resolved against the previous run
    def compare_defects(self, defect_summaries, previous_record):
        unmatched_previous = list(previous_record['defects']) if previous_record else []

        new_defects = []
        persistent_defects = []
        for summary in defect_summaries:
            position = summary['world_position']
            closest_index = None
            closest_distance = MATCH_RADIUS_METERS
            for index, previous_defect in enumerate(unmatched_previous):
                distance = math.sqrt(
                    (position.x - previous_defect['north']) ** 2
                    + (position.y - previous_defect['east']) ** 2
                    + (position.z - previous_defect['altitude']) ** 2)
                if distance <= closest_distance:
                    closest_distance = distance
                    closest_index = index

            if closest_index is None:
                new_defects.append(summary)
            else:
                persistent_defects.append(summary)
                unmatched_previous.pop(closest_index)

        return new_defects, persistent_defects, unmatched_previous

    # Write report.md and move the photo directory into a timestamped folder
    def write_report(self):
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        os.makedirs('reports/current', exist_ok=True)
        report_directory = f'reports/{timestamp}'
        shutil.move('reports/current', report_directory)

        defect_summaries = []
        for defect_event in self.defect_events:
            defect_summaries.append({
                'label': defect_event.label,
                'confidence': defect_event.confidence,
                'world_position': defect_event.world_position,
                'photo_path': os.path.join(
                    report_directory, 'photos',
                    os.path.basename(defect_event.image_path)),
            })

        previous_record = self.load_previous_defects()
        new_defects, persistent_defects, resolved_defects = self.compare_defects(
            defect_summaries, previous_record)
        self.write_defects_json(report_directory, defect_summaries)

        findings_text = self.request_findings(defect_summaries)

        duration_seconds = time.time() - self.takeoff_time if self.takeoff_time else 0.0
        if findings_text is not None:
            model_used = self.llm_model or PROVIDER_DEFAULT_MODELS.get(self.llm_provider)
            provider_used = self.llm_provider
        else:
            model_used = 'none'
            provider_used = 'none'

        if previous_record is None:
            changes_lines = [
                '## Changes',
                '',
                'This is the first inspection of this site. '
                'There is no previous mission to compare.',
            ]
        else:
            changes_lines = [
                f"## Changes since {previous_record['timestamp']}",
                '',
                '| Status | Label | North | East | Altitude |',
                '|---|---|---|---|---|',
            ]
            for summary in new_defects:
                position = summary['world_position']
                changes_lines.append(
                    f"| new | {summary['label']} | {position.x:.2f} | "
                    f"{position.y:.2f} | {position.z:.2f} |")
            for summary in persistent_defects:
                position = summary['world_position']
                changes_lines.append(
                    f"| persistent | {summary['label']} | {position.x:.2f} | "
                    f"{position.y:.2f} | {position.z:.2f} |")
            for previous_defect in resolved_defects:
                changes_lines.append(
                    f"| resolved | {previous_defect['label']} | "
                    f"{previous_defect['north']:.2f} | "
                    f"{previous_defect['east']:.2f} | "
                    f"{previous_defect['altitude']:.2f} |")

        if self.health_alerts:
            health_lines = [
                '## Health',
                '',
                '| Component | Score | Threshold | Message |',
                '|---|---|---|---|',
            ]
            for alert in self.health_alerts:
                health_lines.append(
                    f'| {alert.component} | {alert.anomaly_score:.3f} | '
                    f'{alert.threshold:.3f} | {alert.message} |')
        else:
            health_lines = [
                '## Health',
                '',
                'No health anomaly was detected during this mission.',
            ]

        report_lines = [
            f'# Inspection report, {time.strftime("%Y-%m-%d")}',
            '',
            '| Start | Duration | Defects found | Provider | Model |',
            '|---|---|---|---|---|',
            f'| {time.strftime("%Y-%m-%d %H:%M:%S")} | '
            f'{duration_seconds:.1f}s | {len(defect_summaries)} | '
            f'{provider_used} | {model_used} |',
            '',
            '## Findings',
            '',
            findings_text or (
                'No findings are available. '
                'The system found no API key, or the request failed.'),
            '',
            *changes_lines,
            '',
            *health_lines,
            '',
            '## Appendix, raw detections',
            '',
            '| Label | Confidence | North | East | Altitude | Photo |',
            '|---|---|---|---|---|---|',
        ]
        for summary in defect_summaries:
            position = summary['world_position']
            photo_relative = os.path.join(
                'photos', os.path.basename(summary['photo_path']))
            report_lines.append(
                f"| {summary['label']} | {summary['confidence']:.2f} | "
                f"{position.x:.2f} | {position.y:.2f} | {position.z:.2f} | "
                f"![{summary['label']}]({photo_relative}) |")

        with open(os.path.join(report_directory, 'report.md'), 'w') as report_file:
            report_file.write('\n'.join(report_lines) + '\n')

        self.get_logger().info(f'report written to {report_directory}/report.md')


# Start the node and spin
def main():
    rclpy.init()
    report_writer = ReportWriter()
    rclpy.spin(report_writer)
    report_writer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
