import re, sys, collections
CAT = [('TTC', r'EStop: TTC'), ('입력(역방향속도)', r'invalid_or_reverse_speed'), ('TRAILING무목표', r'trailing_without_target'),
       ('엔벨로프(추종/방향)', r'outside_(tracking|heading)_envelope|nonlinear_prediction_outside'),
       ('제동엔벨로프', r'braking_envelope'), ('OSQP계산실패', r'STOP requested: osqp'), ('계산시간초과', r'STOP requested: deadline_exceeded'),
       ('기타STOP', r'STOP requested')]
OK = [('연속계획(계산miss흡수)', r'plan_continuation'), ('지연해 사용', r'_late'), ('braking_recovery', r'braking_recovery')]
for f in sys.argv[1:]:
    auto = False; c = collections.Counter(); ok = collections.Counter(); t0 = None; tauto = 0.; last = None
    for line in open(f, errors='ignore'):
        m = re.search(r'\[(\d{9,10}\.\d+)\]', line)
        if not m: continue
        t = float(m.group(1))
        if 'autodrive engaged' in line: auto = True; t0 = t; continue
        if auto and ('manual drive' in line or 'ESTOP LATCHED' in line or 'released -> none' in line):
            tauto += t - t0; auto = False; continue
        if not auto: continue
        last = t
        for n, p in CAT:
            if re.search(p, line): c[n] += 1; break
        else:
            for n, p in OK:
                if re.search(p, line): ok[n] += 1; break
    if auto and last: tauto += last - t0
    print(f.split('/')[-1], f'자율 {tauto:.0f}s', dict(c.most_common()), '| 비정지:', dict(ok))
