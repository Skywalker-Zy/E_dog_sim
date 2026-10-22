from dataclasses import asdict
from pathlib import Path
from datetime import datetime
import csv
import json
import math
from PIL import Image


def timestamp():
    # User's local timezone is Asia/Shanghai.
    from zoneinfo import ZoneInfo
    return datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')


def save_snapshot(directory, image, result):
    directory = Path(directory) / ('snapshot_' + timestamp())
    directory.mkdir(parents=True)
    Image.fromarray(image).save(directory / 'image.png')
    (directory / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    (directory / 'settings.json').write_text(json.dumps(result['settings'], indent=2) + '\n')
    return directory


def save_scan(directory, records, settings, label='scan'):
    directory = Path(directory) / (label + '_' + timestamp())
    directory.mkdir(parents=True)
    found = sorted({i for row in records for i in row['decoded_indices']})
    summary = dict(settings=asdict(settings), frames=len(records), decoded_indices=found,
                   missing_indices=sorted(set(range(1, 36)) - set(found)), coverage=len(found) / 35,
                   max_single_frame=max((len(r['decoded_indices']) for r in records), default=0),
                   processing_seconds=sum(r['processing_seconds'] for r in records))
    (directory / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    (directory / 'frames.json').write_text(json.dumps(records, ensure_ascii=False, indent=2) + '\n')
    with (directory / 'frames.csv').open('w', newline='') as stream:
        fields = ['frame','x_m','height_m','yaw_deg','pitch_deg','hfov_deg','vfov_deg','width','height','decoded_count','decoded_indices','processing_seconds']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, row in enumerate(records):
            values = {key: row['settings'][key] for key in ['x_m','height_m','yaw_deg','pitch_deg','width','height']}
            values.update(frame=index + 1, hfov_deg=row['hfov_deg'], vfov_deg=row['vfov_deg'], decoded_count=len(row['decoded_indices']), decoded_indices=';'.join(map(str,row['decoded_indices'])), processing_seconds=row['processing_seconds'])
            writer.writerow(values)
    return directory, summary


def save_sweep(directory, comparisons):
    directory = Path(directory) / ('sweep_' + timestamp())
    directory.mkdir(parents=True)
    (directory / 'comparison.json').write_text(json.dumps(comparisons, indent=2) + '\n')
    with (directory / 'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['width','height','hfov_deg','decoded_count','total','coverage','max_single_frame','frames','processing_seconds','missing_indices','report'])
        writer.writeheader()
        for row in comparisons:
            writer.writerow({**row,'missing_indices':';'.join(map(str,row['missing_indices']))})
    return directory
