/*
 * Wuji SDK C - local SDK user management.
 *
 * Calibration data belongs to the current SDK user. Run this example before
 * calibration when you need to inspect, create, update, or switch that user.
 *
 * Run: ./build/3_user_management
 *      ./build/3_user_management --list
 *      ./build/3_user_management --user-name Alice --user-description "right hand"
 *      ./build/3_user_management --default-user
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wuji_sdk.h"

typedef struct ExampleArgs {
    const char *user_name;
    const char *user_description;
    bool default_user;
    bool list;
    int log_level;
} ExampleArgs;

static const char *optional_text(const char *value) {
    return value ? value : "None";
}

static void print_usage(FILE *stream, const char *program) {
    fprintf(
        stream,
        "Usage: %s [--user-name name] [--user-description text]\n"
        "          [--default-user] [--list]\n"
        "          [--log-level trace|debug|info|warn|warning|error|off]\n",
        program);
}

static int parse_log_level(const char *value, int *out) {
    if (strcmp(value, "trace") == 0) *out = 5;
    else if (strcmp(value, "debug") == 0) *out = 4;
    else if (strcmp(value, "info") == 0) *out = 3;
    else if (strcmp(value, "warn") == 0 || strcmp(value, "warning") == 0) *out = 2;
    else if (strcmp(value, "error") == 0) *out = 1;
    else if (strcmp(value, "off") == 0) *out = 0;
    else return -1;
    return 0;
}

static int option_value(int argc, char **argv, int *index, const char **out) {
    if (*index + 1 >= argc) return -1;
    *out = argv[++(*index)];
    return 0;
}

static int parse_args(int argc, char **argv, ExampleArgs *args) {
    *args = (ExampleArgs){ .log_level = 2 };
    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const char *value = NULL;
        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
            print_usage(stdout, argv[0]);
            return 1;
        } else if (strcmp(arg, "--user-name") == 0) {
            if (option_value(argc, argv, &i, &args->user_name) != 0) return -1;
        } else if (strcmp(arg, "--user-description") == 0) {
            if (option_value(argc, argv, &i, &args->user_description) != 0) return -1;
        } else if (strcmp(arg, "--default-user") == 0) {
            args->default_user = true;
        } else if (strcmp(arg, "--list") == 0) {
            args->list = true;
        } else if (strcmp(arg, "--log-level") == 0) {
            if (option_value(argc, argv, &i, &value) != 0 ||
                parse_log_level(value, &args->log_level) != 0) {
                return -1;
            }
        } else {
            return -1;
        }
    }
    if (args->default_user && args->user_name) return -1;
    if (args->user_description && !args->user_name) return -1;
    return 0;
}

static void print_user(const char *label, const WujiUserInfo *user) {
    printf("%s:\n", label);
    printf("  user_id:      '%s'\n", optional_text(user->user_id));
    printf("  display_name: %s\n", optional_text(user->display_name));
    printf("  description:  %s\n", optional_text(user->description));
    printf("  is_default:   %s\n", user->is_default ? "True" : "False");
}

static int report_error(const char *operation, WujiStatus status) {
    if (status == WUJI_STATUS_OK) return 0;
    fprintf(stderr, "%s failed: %s\n", operation, optional_text(wuji_last_error()));
    return -1;
}

static int switch_or_create_user(const ExampleArgs *args, WujiUserInfo *out_user) {
    if (args->default_user) {
        return report_error(
            "wuji_switch_to_default_user",
            wuji_switch_to_default_user(out_user));
    }
    if (!args->user_name) {
        return report_error("wuji_current_user", wuji_current_user(out_user));
    }

    WujiUserInfo *users = NULL;
    size_t user_count = 0;
    WujiStatus status = wuji_list_users(&users, &user_count);
    if (report_error("wuji_list_users", status) != 0) return -1;

    size_t match_index = 0;
    size_t match_count = 0;
    for (size_t i = 0; i < user_count; i++) {
        if (users[i].display_name && strcmp(users[i].display_name, args->user_name) == 0) {
            match_index = i;
            match_count++;
        }
    }
    if (match_count > 1) {
        fprintf(stderr, "Multiple SDK users are named '%s'; use a unique display name.\n",
                args->user_name);
        wuji_user_info_array_free(users, user_count);
        return -1;
    }

    if (match_count == 1) {
        const WujiUserInfo *matched = &users[match_index];
        bool update_description = args->user_description &&
                                  (!matched->description ||
                                   strcmp(matched->description, args->user_description) != 0);
        if (update_description) {
            WujiUserUpdate update = { .description = args->user_description };
            WujiUserInfo updated = {0};
            status = wuji_update_user(matched->user_id, &update, &updated);
            if (status == WUJI_STATUS_OK) {
                status = wuji_switch_user(updated.user_id, out_user);
                wuji_user_info_free(&updated);
            }
        } else {
            status = wuji_switch_user(matched->user_id, out_user);
        }
        wuji_user_info_array_free(users, user_count);
        return report_error("select existing SDK user", status);
    }

    wuji_user_info_array_free(users, user_count);
    WujiUserInfo created = {0};
    status = wuji_create_user(
        args->user_name, args->user_description, NULL, &created);
    if (report_error("wuji_create_user", status) != 0) return -1;
    status = wuji_switch_user(created.user_id, out_user);
    wuji_user_info_free(&created);
    return report_error("wuji_switch_user", status);
}

static int print_user_list(const char *current_user_id) {
    WujiUserInfo *users = NULL;
    size_t user_count = 0;
    WujiStatus status = wuji_list_users(&users, &user_count);
    if (report_error("wuji_list_users", status) != 0) return -1;

    printf("\nSDK users:\n");
    for (size_t i = 0; i < user_count; i++) {
        bool is_current = users[i].user_id && current_user_id &&
                          strcmp(users[i].user_id, current_user_id) == 0;
        printf(
            "%c %s user_id='%s' description=%s\n",
            is_current ? '*' : ' ',
            optional_text(users[i].display_name),
            optional_text(users[i].user_id),
            optional_text(users[i].description));
    }
    wuji_user_info_array_free(users, user_count);
    return 0;
}

int main(int argc, char **argv) {
    ExampleArgs args;
    int parse_status = parse_args(argc, argv, &args);
    if (parse_status != 0) {
        if (parse_status < 0) print_usage(stderr, argv[0]);
        return parse_status < 0 ? 2 : 0;
    }

    WujiInitOptions init = { .log_level = args.log_level };
    if (report_error("wuji_init", wuji_init(&init)) != 0) return 1;

    WujiUserInfo previous = {0};
    WujiUserInfo current = {0};
    if (report_error("wuji_current_user", wuji_current_user(&previous)) != 0 ||
        switch_or_create_user(&args, &current) != 0) {
        wuji_user_info_free(&previous);
        wuji_shutdown();
        return 1;
    }

    print_user("previous current user", &previous);
    print_user("current SDK user", &current);
    int result = args.list && print_user_list(current.user_id) != 0 ? 1 : 0;

    wuji_user_info_free(&previous);
    wuji_user_info_free(&current);
    wuji_shutdown();
    return result;
}
