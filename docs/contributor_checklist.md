# Contributor Checklist

Use this checklist for every new wrapper method.

## Required

- Public API name is `snake_case`.
- Inputs are validated with clear error messages.
- Defaults are safe for a classroom environment.
- Blocking behavior is explicit and consistent.
- Return type matches the rest of the module.
- Critical-path failures do not fail silently.
- One classroom-ready example is added to docs.
- `docs/api_reference.md` is updated.
- `docs/coverage_matrix.md` is updated.
- A regression test is added or updated.

## Behavior contracts

- Position actions block by default unless clearly documented otherwise.
- Streaming and subscriptions do not block.
- Wrapper methods should return SDK actions unchanged when `blocking=False`.
- Wrapper methods should raise `ValueError` or `TypeError` for invalid user input rather than printing warnings and continuing.

## Review questions

- Does the new method read like a student intention rather than an SDK transport detail?
- Can a learner use it correctly without memorizing hidden units?
- If the underlying SDK feature is too low-level, should it be marked `intentionally_omitted` instead?
