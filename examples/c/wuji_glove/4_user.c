/*
 * Purpose:
 *   Manage local SDK users and export or import their data.
 * Prerequisites:
 *   Install the C SDK. No device is required.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 4_user.
 *   ./build/glove/4_user
 *   ./build/glove/4_user --list
 *   ./build/glove/4_user --user-name Alice --user-description "right hand"
 *   ./build/glove/4_user --default-user
 *   ./build/glove/4_user --delete-user USER_ID --list
 *   ./build/glove/4_user --export user_data.zip
 *   ./build/glove/4_user --export user_data.zip --user-id USER_ID
 *   ./build/glove/4_user --preview user_data.zip
 *   ./build/glove/4_user --import user_data.zip
 * Expected Behavior:
 *   Print the previous and selected SDK user. The selection remains active.
 *   Use --list, --user-name NAME, --user-description TEXT, or --default-user.
 *   --delete-user USER_ID deletes that user and its stored data.
 *   Deleting the current user switches to the default user.
 *   Export hand models and tactile calibration data for the current user,
 *   unless --user-id selects another user. Preview validates without writing.
 *   Import restores the bundle owner's data, creating that user if needed and
 *   overwriting matching files. It does not switch the current user.
 *   Data operations print confirmation instead of user selection details.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */
#include <stdbool.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wuji_sdk.h"

static volatile sig_atomic_t stop_requested = 0;
static void on_sigint(int signum) {
    (void)signum;
    stop_requested = 1;
}

typedef struct ExampleArgs {
    const char *user_name;
    const char *delete_user;
    const char *user_description;
    const char *export_path;
    const char *preview_path;
    const char *import_path;
    const char *user_id;
    bool default_user;
    bool list;
} ExampleArgs;

static const char *optional_text(const char *value) {
    return value ? value : "None";
}

static void print_usage(FILE *stream, const char *program) {
    fprintf(
        stream,
        "Usage: %s [--user-name name] [--user-description text]\n"
        "          [--default-user | --delete-user USER_ID] [--list]\n"
        "       %s (--export FILE.zip [--user-id USER_ID] | --preview FILE.zip | --import FILE.zip)\n",
        program,
        program);
}

static int option_value(int argc, char **argv, int *index, const char **out) {
    if (*index + 1 >= argc || strncmp(argv[*index + 1], "--", 2) == 0) return -1;
    *out = argv[++(*index)];
    return 0;
}

static int parse_args(int argc, char **argv, ExampleArgs *args) {
    *args = (ExampleArgs){0};
    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        if (strcmp(arg, "--user-name") == 0) {
            if (option_value(argc, argv, &i, &args->user_name) != 0) return -1;
        } else if (strcmp(arg, "--user-description") == 0) {
            if (option_value(argc, argv, &i, &args->user_description) != 0) return -1;
        } else if (strcmp(arg, "--delete-user") == 0) {
            if (option_value(argc, argv, &i, &args->delete_user) != 0) return -1;
        } else if (strcmp(arg, "--export") == 0) {
            if (option_value(argc, argv, &i, &args->export_path) != 0) return -1;
        } else if (strcmp(arg, "--preview") == 0) {
            if (option_value(argc, argv, &i, &args->preview_path) != 0) return -1;
        } else if (strcmp(arg, "--import") == 0) {
            if (option_value(argc, argv, &i, &args->import_path) != 0) return -1;
        } else if (strcmp(arg, "--user-id") == 0) {
            if (option_value(argc, argv, &i, &args->user_id) != 0) return -1;
        } else if (strcmp(arg, "--default-user") == 0) {
            args->default_user = true;
        } else if (strcmp(arg, "--list") == 0) {
            args->list = true;
        } else {
            return -1;
        }
    }
    if (args->user_name && args->user_name[0] == '\0') return -1;
    if (args->delete_user && args->delete_user[0] == '\0') return -1;
    int data_actions = (args->export_path != NULL) + (args->preview_path != NULL) +
                       (args->import_path != NULL);
    int user_actions = (args->user_name != NULL) + args->default_user + (args->delete_user != NULL);
    if (data_actions + user_actions > 1) return -1;
    if (data_actions && args->list) return -1;
    if (args->user_id && !args->export_path) return -1;
    if ((args->export_path && args->export_path[0] == '\0') ||
        (args->preview_path && args->preview_path[0] == '\0') ||
        (args->import_path && args->import_path[0] == '\0')) return -1;
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

static void report_sdk_error(const char *operation) {
    const char *error = wuji_last_error();
    fprintf(stderr, "%s failed: %s\n", operation, error ? error : "Unknown SDK error");
}

static int switch_or_create_user(const ExampleArgs *args, WujiUserInfo *out_user) {
    if (args->default_user) {
        if (wuji_switch_to_default_user(out_user) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_switch_to_default_user");
            return -1;
        }
        return 0;
    }
    if (!args->user_name) {
        if (wuji_current_user(out_user) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_current_user");
            return -1;
        }
        return 0;
    }

    WujiUserInfo *users = NULL;
    size_t user_count = 0;
    WujiStatus status = wuji_list_users(&users, &user_count);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_list_users");
        return -1;
    }

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
        if (status != WUJI_STATUS_OK) {
            report_sdk_error("select existing SDK user");
            return -1;
        }
        return 0;
    }

    wuji_user_info_array_free(users, user_count);
    WujiUserInfo created = {0};
    status = wuji_create_user(
        args->user_name, args->user_description, NULL, &created);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_create_user");
        return -1;
    }
    status = wuji_switch_user(created.user_id, out_user);
    wuji_user_info_free(&created);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_switch_user");
        return -1;
    }
    return 0;
}

static int print_user_list(const char *current_user_id) {
    WujiUserInfo *users = NULL;
    size_t user_count = 0;
    WujiStatus status = wuji_list_users(&users, &user_count);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_list_users");
        return -1;
    }

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

    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno)); return 1;
    }
    if (wuji_init(NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_init");
        return 1;
    }

    WujiUserInfo previous = {0};
    WujiUserInfo current = {0};
    if (stop_requested) { wuji_shutdown(); return 0; }
    int result = 0;
    if (args.export_path != NULL) {
        const char *user_id = args.user_id;
        if (user_id == NULL) {
            if (wuji_current_user(&current) != WUJI_STATUS_OK) {
                report_sdk_error("wuji_current_user");
                result = 1; goto cleanup;
            }
            user_id = current.user_id;
        }
        if (stop_requested) goto cleanup;
        if (wuji_user_data_export(user_id, args.export_path) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_user_data_export");
            result = 1; goto cleanup;
        }
        printf("Exported user data: %s\n", args.export_path);
        goto cleanup;
    }
    if (args.preview_path != NULL) {
        if (wuji_user_data_preview(args.preview_path) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_user_data_preview");
            result = 1; goto cleanup;
        }
        printf("Validated user data: %s\n", args.preview_path);
        goto cleanup;
    }
    if (args.import_path != NULL) {
        if (wuji_user_data_import(args.import_path) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_user_data_import");
            result = 1; goto cleanup;
        }
        printf("Imported user data: %s\n", args.import_path);
        goto cleanup;
    }
    if (wuji_current_user(&previous) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_current_user");
        result = 1; goto cleanup;
    }
    if (stop_requested) goto cleanup;
    if (args.delete_user != NULL) {
        if (wuji_delete_user(args.delete_user) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_delete_user");
            result = 1; goto cleanup;
        }
        printf("Deleted user: %s\n", args.delete_user);
    }
    if (stop_requested) goto cleanup;
    if (switch_or_create_user(&args, &current) != 0) {
        result = 1; goto cleanup;
    }

    print_user("previous current user", &previous);
    print_user("current SDK user", &current);
    result = args.list && print_user_list(current.user_id) != 0 ? 1 : 0;

cleanup:
    wuji_user_info_free(&previous);
    wuji_user_info_free(&current);
    wuji_shutdown();
    return result;
}
