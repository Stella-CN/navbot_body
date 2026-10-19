#!/usr/bin/env python3
"""Generate in a staging copy, audit, then publish. Never build or access a probe."""
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MX = '/Applications/STMicroelectronics/STM32CubeMX.app/Contents/Resources'


def user_regions(root):
    result = {}
    for source in (root / 'Core').rglob('*'):
        if source.suffix in ('.c', '.h'):
            result[str(source.relative_to(root))] = [(name, body.strip()) for name, body in re.findall(
                r'/\* USER CODE BEGIN ([^*]+) \*/(.*?)/\* USER CODE END \1 \*/',
                source.read_text(), re.S)]
    return result


def protected_files(root):
    files = [root / name for name in ('CMakeLists.txt', '.clang-format', 'openocd.cfg',
                                      'navbot_body.code-workspace', 'README.md', 'CMakePresets.json')]
    for directory in ('Applications', 'BSP', '.vscode', 'tools'):
        files.extend(path for path in (root / directory).rglob('*') if path.is_file())
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files}


def classify_diagnostics(log):
    """Only known, source-audited generator diagnostics are tolerated."""
    groups = {}
    unknown = []
    for line in log.splitlines():
        if not any(token in line for token in ('[WARN]', '[ERROR]', 'OptionalMessage_ERROR')):
            continue
        if 'OptionalMessage_ERROR' in line:
            if re.search(r'IP \(DMA\).*Id \(USART3_(RX|TX)\) currently not set', line):
                category = 'dma_import_order'
            elif 'Pin37 (VP_RIF_VS_RIF1) cannot be retrieved for this MCU' in line:
                category = 'unselected_rif_virtual_pin'
            else:
                unknown.append(line)
                continue
        elif '[ERROR]' in line:
            if 'Engine:' in line and '.pdsc: PDSC version is not supported' in line:
                category = 'unselected_expansion_pack'
            else:
                unknown.append(line)
                continue
        elif 'IP:' in line and ('configHEAP_CLEAR_MEMORY_ON_FREE not found' in line
                                or 'INCLUDE_xTaskDelayUntil not found' in line):
            category = 'freertos_database_version'
        elif 'FreeRTOS:' in line and 'Null or empty value:' in line:
            category = 'empty_optional_rtos_table'
        elif 'SupportedApi:' in line and 'Cannot load RTOS API schema' in line:
            category = 'rtos_api_schema'
        elif re.search(r'\[WARN\] (ApiDb|DbFile|IntegrityCheckThread|PackLoader|ConditionMgr|ModelIntegratedComponent|ModelPack|ThirdParty):', line):
            category = 'generator_environment_warning'
        else:
            unknown.append(line)
            continue
        groups.setdefault(category, {'count': 0, 'sample': line})['count'] += 1
    return {'categories': groups, 'unresolved_errors': unknown}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cubemx-dir', type=Path, default=Path(DEFAULT_MX))
    args = parser.parse_args()
    mx = args.cubemx_dir.resolve()
    java, jar = mx / 'jre/Contents/Home/bin/java', mx / 'STM32CubeMX'
    if not java.is_file() or not jar.is_file():
        parser.error('Expected macOS CubeMX Resources directory with bundled JRE and STM32CubeMX')
    previous_regions = user_regions(ROOT)
    previous_protected = protected_files(ROOT)
    with tempfile.TemporaryDirectory(prefix='navbot-cubemx-') as directory:
        stage = Path(directory) / 'navbot_body'
        shutil.copytree(ROOT, stage, ignore=shutil.ignore_patterns('build', '*.log', '__pycache__'))
        script = '\n'.join([
            f'config load "{stage / "navbot_body.ioc"}"', 'project name navbot_body',
            'project toolchain CMake', 'project compiler GCC', 'project couplefilesbyip 1',
            'project generate', f'config save "{stage / "navbot_body.ioc"}"',
            'isPluginError "Clock Configuration"', 'isPluginError "Pinout & Configuration"', 'exit', ''])
        script_path = Path(directory) / 'generate.script'
        script_path.write_text(script)
        result = subprocess.run([str(java), '-jar', str(jar), '-q', str(script_path)],
                                cwd=mx, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, check=False, timeout=180)
        log_dir = ROOT / 'validation'
        log_dir.mkdir(exist_ok=True)
        (log_dir / 'cubemx-generation.log').write_text(result.stdout)
        diagnostics = classify_diagnostics(result.stdout)
        (log_dir / 'generator-diagnostics.json').write_text(json.dumps(diagnostics, indent=2) + '\n')
        required = ['Clock Configuration Error: false', 'Pinout & Configuration Error: false']
        if (result.returncode or re.search(r'^KO$', result.stdout, re.M)
                or not all(text in result.stdout for text in required)
                or diagnostics['unresolved_errors']):
            raise SystemExit('CubeMX audit failed; source files unchanged. Inspect validation diagnostics.')
        if user_regions(stage) != previous_regions or protected_files(stage) != previous_protected:
            raise SystemExit('CubeMX changed maintained code; source files unchanged.')
        subprocess.run([sys.executable, str(stage / 'tools/static_validate.py')], check=True)
        # Publish only after all static assertions and preservation checks pass.
        # No files are removed from the destination by this copy operation.
        for path in stage.rglob('*'):
            if not path.is_file() or 'validation' in path.relative_to(stage).parts:
                continue
            destination = ROOT / path.relative_to(stage)
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists() or path.read_bytes() != destination.read_bytes():
                shutil.copy2(path, destination)
    print('CubeMX generated and audited in staging; USER CODE and maintained files preserved.')
    final = subprocess.run([sys.executable, str(ROOT / 'tools/static_validate.py')],
                           text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    (ROOT / 'validation/STATIC_CHECKS.txt').write_text(final.stdout)
    print(final.stdout, end='')
    if final.returncode:
        raise SystemExit(final.returncode)
    (ROOT / 'validation/regeneration-check.json').write_text(json.dumps({
        'result': 'PASS', 'clock_plugin_error': False, 'pinout_plugin_error': False,
        'core_files_with_user_regions_preserved': len(previous_regions),
        'maintained_files_preserved': len(previous_protected),
        'unresolved_generator_errors': 0, 'compile_or_probe_invoked': False
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
