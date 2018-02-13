from __future__ import absolute_import, print_function, unicode_literals

import os
import pytest
import hypothesis


if hasattr(hypothesis, 'HealthCheck') and hasattr(hypothesis.HealthCheck, 'too_slow'):
    hypothesis.settings.register_profile("travis", hypothesis.settings(
        suppress_health_check=[getattr(hypothesis.HealthCheck, 'too_slow')]
    ))
else:
    hypothesis.settings.register_profile("travis", hypothesis.settings(
        # default
    ))

hypothesis.settings.register_profile("dev", hypothesis.settings(
    verbosity=hypothesis.Verbosity.verbose,
))

# default settings
hypothesis.settings.load_profile(os.getenv('HYPOTHESIS_PROFILE', 'dev'))
